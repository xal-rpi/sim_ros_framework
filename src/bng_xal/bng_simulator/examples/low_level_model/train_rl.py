import random
# import time
from typing import Any, Tuple, Callable, Dict, NamedTuple
# from functools import partial

import jax
import jax.numpy as jnp

import optax
from flax.training.train_state import TrainState

import numpy as np

from environment import Environment, EnvParams
from wrappers import NormalizeVecObservation, NormalizeVecReward
from logging_utils import TrainCheckpoints

from engine_model import EngineModelEnv

class Transition(NamedTuple):
    """
    Define the transition tuple for the PPO agent
    """
    done: jnp.ndarray
    action: jnp.ndarray
    value: jnp.ndarray
    reward: jnp.ndarray
    log_prob: jnp.ndarray
    obs: jnp.ndarray
    info: jnp.ndarray

def get_evaluate_env_fn(
    env : Environment,
    env_params : EnvParams,
    init_network : Callable[[jnp.ndarray,], Tuple],
    num_envs : int,
    max_iter : int
):
    """
    Return a function to evaluate the policies in the environment and
    output quantities of interest for logging.

    Returns:
        A function to evaluate the policy in the environment
        Callable[[EnvParams, Any, jnp.ndarray], dict]
    """
    # Get the policy function and the initialization parameters for jit
    init_params, policy_fn, _ = init_network(jax.random.PRNGKey(0))

    @jax.jit
    def sim_env(env_params, policy_params, rng):
        """
        Simulate a policy in the Gym environment.
        """
        # Initialize the environments
        rng, _rng = jax.random.split(rng)
        reset_rng = jax.random.split(_rng, num_envs)
        obsv, env_state = env.reset(reset_rng, env_params)

        def _step_env(running_state, _):
            """
            One step simulation function to use in a scan.
            """
            obsv, env_state, rng = running_state

            rng, _rng = jax.random.split(rng)
            action = policy_fn(policy_params, obsv, _rng)

            rng, _rng = jax.random.split(rng)
            rng_step = jax.random.split(_rng, num_envs)
            obsv, env_state, reward, done, info = env.step(
                rng_step, env_state, action, env_params
            )
            # truncated = info["truncation_info"]["truncated"]
            truncated = info["do_terminate"]
            done = jnp.logical_or(done, truncated)
            actual_done = info["actual_done"]

            # Get relevant quantities for logging from info
            log_info = env.get_logging_info(info)
            running_state = (obsv, env_state, rng)
            return running_state, (reward, done, actual_done, log_info)

        running_state = (obsv, env_state, rng)
        _, (reward, done, actual_done, log_info) = jax.lax.scan(
            _step_env, running_state, None, max_iter
        )

        # Now let's first parse the information by only keeping the values
        # prior to the first done. We need to do this because the scan function
        # keeps running even if done is true.
        done_sum = jnp.cumsum(done, axis=0)
        idx_interest = done_sum < 1
        reward = jnp.where(idx_interest, reward, 0.0)
        log_info = jax.tree_util.tree_map(
            lambda x: jnp.where(
                idx_interest if x.ndim == 2 else idx_interest[..., None],
                x, 0.0
            ),
            log_info
        )
        actual_done_sum = jnp.cumsum(actual_done, axis=0)
        indx_actual_done = actual_done_sum < 1
        episode_length_avg = jnp.mean(jnp.sum(indx_actual_done, axis=0))

        # Now let's compute the return, min max mean and length
        return_value = jnp.sum(reward, axis=0)
        return_value_mean = jnp.mean(return_value)
        return_value_min = jnp.min(return_value)
        return_value_max = jnp.max(return_value)

        # Now proceed with the log info to compute their average
        log_info = jax.tree_util.tree_map(
            lambda x: jnp.mean(jnp.sum(x, axis=0), axis=0), log_info
        )

        # Output dictionary
        res_dict = {
            'Reward/mean': return_value_mean,
            'Reward/min': return_value_min,
            'Reward/max': return_value_max,
            'Reward/episode_length_avg': episode_length_avg,
            **{
                f'Info/{k}': v for k, v in log_info.items()
            }
        }
        return res_dict

    # TODO: This is needed if we want to call in in async
    # Call this function once at the beginning to compile it
    res = sim_env( env_params, init_params, jax.random.PRNGKey(0))
    jax.block_until_ready(res)
    return sim_env


def make_train(
    env : EngineModelEnv,
    env_params : EnvParams,
    init_policy : Callable[[jnp.ndarray,], Tuple],
    configs : Dict[str, Any],
    save_name : str,
    seed : int =0,
    base_logs_dir : str = "training_files/rl/"
):
    """A function to design the training function for the cost2go
    """
    # Set the random seed
    random.seed(seed)
    np.random.seed(seed)

    # Get the actor and critic networks
    init_network = init_policy

    # Extract the relavant training parameters
    rl_config = configs['rl_agent']
    config = rl_config # Backward compatibility

    # Function for evaluating the environment and logging
    ep_sim_fn = get_evaluate_env_fn(
        env, env_params, init_network, rl_config["NUM_ENVS_EVAL"],
        env_params.max_steps_in_episode
    )

    # Automatically set the number of updates
    config["NUM_UPDATES"] = int(
        config["TOTAL_TIMESTEPS"] // config["NUM_STEPS"] // config["NUM_ENVS"]
    )

    # Automatically set the minibatch size
    config["MINIBATCH_SIZE"] = int(
        config["NUM_ENVS"] * config["NUM_STEPS"] // config["NUM_MINIBATCHES"]
    )

    # Define the logging function
    def _log_data(
        _env_params : EnvParams,
        _policy_params : Dict[str, Any],
        _rng : jnp.ndarray,
        extra : dict
    ):
        """
        Log the RL training data
        """
        # Let's simulate the environment
        if not ckpt_model.should_update():
            ckpt_model.write_checkpoint_and_log_data(
                None, extra, step_factor = rl_config["NUM_ENVS"]
            )
            return

        # Let's simulate the environment
        sim_env_res = ep_sim_fn(_env_params, _policy_params, _rng)

        # Now let's add the extra information and save it
        rew_sim_env_res = {**sim_env_res, **extra}
        sim_env_res = {'learned_agent_params': _policy_params, **rew_sim_env_res}
        ckpt_model.write_checkpoint_and_log_data(
            sim_env_res, rew_sim_env_res, step_factor = rl_config["NUM_ENVS"]
        )

    # Wrapper around the environment for logging
    # env = LogWrapper(env)

    if config["NORMALIZE_ENV"]:
        env = NormalizeVecObservation(env)
        env = NormalizeVecReward(env, config["GAMMA"])

    def linear_schedule(count : int):
        """
        Define a linear schedule for the learning rate
        """
        frac = (
            1.0
            - (count // (config["NUM_MINIBATCHES"] * config["UPDATE_EPOCHS"]))
            / config["NUM_UPDATES"]
        )
        return config["LR"] * frac

     # Checkpoint object
    ckpt_model = TrainCheckpoints(
        experiment_dir = base_logs_dir,
        output_name = save_name,
        ckpt_cfg = configs['track_n_checkpoints'],
        best_mode = 'max',
        extra_config_to_save_as_yaml = configs,
        saving_freq = rl_config['LOG_EVERY_N_UPDATES']
    )

    def train(rng : jnp.ndarray):
        """
        Main jittable and vectorized training function
        """
        rng, _rng = jax.random.split(rng)
        network_params, _, network = init_network(_rng)

        # Let's store the first simulation
        rng, _rng = jax.random.split(rng)
        jax.debug.callback(_log_data, env_params, network_params, _rng, {})

        if config["ANNEAL_LR"]:
            tx = optax.chain(
                optax.clip_by_global_norm(config["MAX_GRAD_NORM"]),
                optax.adam(learning_rate=linear_schedule, eps=1e-5),
            )
        else:
            tx = optax.chain(
                optax.clip_by_global_norm(config["MAX_GRAD_NORM"]),
                optax.adam(config["LR"], eps=1e-5),
            )

        train_state = TrainState.create(
            apply_fn=network.apply,
            params=network_params,
            tx=tx,
        )

        # INIT ENV
        rng, _rng = jax.random.split(rng)
        reset_rng = jax.random.split(_rng, config["NUM_ENVS"])
        obsv, env_state = env.reset(reset_rng, env_params)

        # TRAIN LOOP
        @scan_tqdm(config["NUM_UPDATES"], print_rate=10)
        def _update_step(runner_state, _):
            """
            One step of the training loop
            """
            # COLLECT TRAJECTORIES
            def _env_step(runner_state, __):
                """
                Environment step function
                """
                train_state, env_state, last_obs, rng = runner_state

                # SELECT ACTION
                rng, _rng = jax.random.split(rng)
                pi, value = network.apply(train_state.params, last_obs)
                action = pi.sample(seed=_rng)
                log_prob = pi.log_prob(action)

                # STEP ENV
                rng, _rng = jax.random.split(rng)
                rng_step = jax.random.split(_rng, config["NUM_ENVS"])
                obsv, env_state, reward, done, info = env.step(
                    rng_step, env_state, action, env_params
                )
                transition = Transition(
                    done, action, value, reward, log_prob,
                    last_obs, info["truncation_info"]["truncated"]
                )
                runner_state = (train_state, env_state, obsv, rng)
                return runner_state, transition

            runner_state, traj_batch = jax.lax.scan(
                _env_step, runner_state, None, config["NUM_STEPS"]
            )

            # CALCULATE ADVANTAGE
            train_state, env_state, last_obs, rng = runner_state
            _, last_val = network.apply(train_state.params, last_obs)

            def _calculate_gae(traj_batch, last_val):
                def _get_advantages(gae_and_next_value, transition):
                    gae, next_value = gae_and_next_value
                    done, value, reward, truncated = (
                        transition.done,
                        transition.value,
                        transition.reward,
                        transition.info,
                    )
                    delta = reward + config["GAMMA"] * next_value * (1 - done) - value
                    gae = (
                        delta
                        + config["GAMMA"] * config["GAE_LAMBDA"] * \
                            (1 - done) * gae
                    )
                    # Skip the truncated steps
                    gae = jnp.where(truncated, 0.0, gae)
                    return (gae, value), gae

                _, advantages = jax.lax.scan(
                    _get_advantages,
                    (jnp.zeros_like(last_val), last_val),
                    traj_batch,
                    reverse=True,
                    unroll=16,
                )
                return advantages, advantages + traj_batch.value

            advantages, targets = _calculate_gae(
                traj_batch, last_val
            )

            # UPDATE NETWORK
            def _update_epoch(update_state, ___):
                """
                Update per epoch function
                """
                def _update_minbatch(train_state, batch_info):
                    """
                    Minibatch update function
                    """
                    traj_batch, advantages, targets = batch_info

                    def _loss_fn(params, traj_batch, gae, targets):
                        """
                        Compute the loss function
                        """
                        # RERUN NETWORK
                        pi, value = network.apply(params, traj_batch.obs)
                        log_prob = pi.log_prob(traj_batch.action)

                        # CALCULATE VALUE LOSS
                        value_pred_clipped = traj_batch.value + (
                            value - traj_batch.value
                        ).clip(-config["CLIP_EPS"], config["CLIP_EPS"])
                        value_losses = jnp.square(value - targets)
                        value_losses_clipped = \
                            jnp.square(value_pred_clipped - targets)
                        value_loss = 0.5 * jnp.maximum(
                            value_losses, value_losses_clipped
                        ).mean()

                        # CALCULATE ACTOR LOSS
                        # jax.debug.print("Log prob: {}", log_prob)
                        # jax.debug.print("Action prob: {}", traj_batch.action)
                        ratio = jnp.exp(log_prob - traj_batch.log_prob)
                        gae = (gae - gae.mean()) / (gae.std() + 1e-8)
                        loss_actor1 = ratio * gae
                        loss_actor2 = (
                            jnp.clip(
                                ratio,
                                1.0 - config["CLIP_EPS"],
                                1.0 + config["CLIP_EPS"],
                            )
                            * gae
                        )
                        loss_actor = -jnp.minimum(loss_actor1, loss_actor2)
                        loss_actor = loss_actor.mean()
                        entropy = 0.0
                        if config["ENT_COEF"] > 0:
                            entropy = pi.entropy().mean()

                        total_loss = (
                            loss_actor
                            + config["VF_COEF"] * value_loss
                            - config["ENT_COEF"] * entropy
                        )
                        ret_val = {
                            'Loss/TotalLoss' : total_loss,
                            'Loss/ValueLoss' : value_loss,
                            'Loss/ActorLoss' : loss_actor,
                            'Loss/Entropy' : entropy
                        }

                        # Compute the L-2 norm of the kernel weights of actor and critic
                        # for regularization
                        if config.get("SPECTRAL_REG_PENALTY", 0) > 0:
                            def jac_regularizer(_obs, _rng, _params):
                                """
                                Compute the jacobian regularization
                                """
                                _pi, _ = network.apply(_params, _obs)
                                mean_val = _pi.mean
                                # Sample a random obs
                                _v_obs = jax.random.normal(_rng, mean_val.shape)
                                _v_obs = _v_obs / (jnp.linalg.norm(_v_obs, axis=-1) + 1e-8)
                                return jnp.sum(mean_val * _v_obs)
                            jac_v = jax.grad(jac_regularizer, argnums=0)
                            jac_v = jax.vmap(jac_v, in_axes=(0, 0, None))
                            num_obs = traj_batch.obs.shape[0] # 128
                            __obs = traj_batch.obs[:num_obs]
                            __rng = jax.random.split(rng, __obs.shape[0])
                            jac_val = jac_v(__obs, __rng, params)
                            jac_val = jnp.sum(jnp.square(jac_val))
                            ret_val["Loss/JacRegCritic"] = jac_val
                            total_loss += config["SPECTRAL_REG_PENALTY"] * jac_val
                            # # Get the kernel weights
                            # reg_loss = 1.0
                            # dict_params = params["params"]
                            # for k, v in dict_params.items():
                            #     if "a_lay" in k or "a_out" in k:
                            #         kweight = v["kernel"]
                            #         reg_loss *= jnp.linalg.norm(kweight, 2)
                            # ret_val["Loss/SpectralRegActor"] = reg_loss
                            # total_loss += config["SPECTRAL_REG_PENALTY"] * reg_loss

                        # return total_loss, (value_loss, loss_actor, entropy)
                        return total_loss, ret_val

                    grad_fn = jax.value_and_grad(_loss_fn, has_aux=True)
                    total_loss, grads = grad_fn(
                        train_state.params, traj_batch, advantages, targets
                    )

                    train_state = train_state.apply_gradients(grads=grads)
                    return train_state, total_loss

                train_state, traj_batch, advantages, targets, rng = update_state
                rng, _rng = jax.random.split(rng)
                batch_size = config["MINIBATCH_SIZE"] * config["NUM_MINIBATCHES"]
                assert (batch_size == config["NUM_STEPS"] * config["NUM_ENVS"]),\
                    "batch size must be equal to number of steps * number of envs"
                permutation = jax.random.permutation(_rng, batch_size)
                batch = (traj_batch, advantages, targets)
                batch = jax.tree_util.tree_map(
                    lambda x: x.reshape((batch_size,) + x.shape[2:]), batch
                )
                shuffled_batch = jax.tree_util.tree_map(
                    lambda x: jnp.take(x, permutation, axis=0), batch
                )
                minibatches = jax.tree_util.tree_map(
                    lambda x: jnp.reshape(
                        x, [config["NUM_MINIBATCHES"], -1] + list(x.shape[1:])
                    ),
                    shuffled_batch,
                )
                train_state, total_loss = jax.lax.scan(
                    _update_minbatch, train_state, minibatches
                )
                update_state = (train_state, traj_batch, advantages, targets, rng)
                _, loss_info = total_loss

                loss_info = jax.tree_util.tree_map(lambda x: x.mean(), loss_info)
                return update_state, loss_info

            update_state = (train_state, traj_batch, advantages, targets, rng)
            update_state, loss_info = jax.lax.scan(
                _update_epoch, update_state, None, config["UPDATE_EPOCHS"]
            )

            # Did the optimization fails? If yes, don't update the prams
            failed_step = jnp.isnan(loss_info["Loss/TotalLoss"]).any()
            train_state = jax.tree_util.tree_map(
                lambda _x, _y : jnp.where(failed_step, _x, _y),
                train_state, update_state[0]
            )
            # metric = traj_batch.info
            rng = update_state[-1]
            rng, _rng = jax.random.split(rng)

            jax.debug.callback(
                _log_data, env_params, train_state.params, _rng, loss_info
            )

            runner_state = (train_state, env_state, last_obs, rng)
            return runner_state, None

        rng, _rng = jax.random.split(rng)
        runner_state = (train_state, env_state, obsv, _rng)
        runner_state, metric = jax.lax.scan(
            _update_step, runner_state, jnp.arange(config["NUM_UPDATES"])
        )
        return {"runner_state": runner_state, "metrics": metric}

    return train, ckpt_model