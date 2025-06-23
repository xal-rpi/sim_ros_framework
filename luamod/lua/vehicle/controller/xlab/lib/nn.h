#pragma once

#ifdef _WIN32
  #define EXPORT __declspec(dllexport)
#else
  #define EXPORT
#endif



typedef enum {
  ACT_NONE = 0,
  ACT_RELU,
  ACT_SIGMOID,
  ACT_TANH,
  ACT_SOFTMAX
} Activation;

typedef struct Model Model;

// allocate a model with num_layers slots
EXPORT Model *nn_create_model(int num_layers);

// configure layer #idx (0‐based) as dense:
//   in_features, out_features
//   weights: row‐major array of size out_features*in_features
//   biases:   array of size out_features
//   activation: one of Activation
// returns 0 on success
EXPORT int nn_set_dense_layer(Model *model, int idx, int in_features, int out_features,
                       const float *weights, const float *biases,
                       Activation activation);

// run inference: input[0..input_size-1]
// out_size ← output vector length, return pointer to new float[out_size]
// must free result with nn_free_buffer
EXPORT float *nn_run(Model *model, const float *input, int input_size, int *out_size);

// free model and its weights
EXPORT void nn_free_model(Model *model);

// free any buffer returned by nn_run
EXPORT void nn_free_buffer(float *buffer);

