#include "nn.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  int in_features;
  int out_features;
  float *weights; // size = out_features*in_features
  float *biases;  // size = out_features
  Activation act;
} DenseLayer;

struct Model {
  int num_layers;
  DenseLayer *layers;
};

Model *nn_create_model(int num_layers) {
  Model *m = malloc(sizeof(Model));
  if (!m)
    return NULL;
  m->num_layers = num_layers;
  m->layers = calloc(num_layers, sizeof(DenseLayer));
  if (!m->layers) {
    free(m);
    return NULL;
  }
  return m;
}

int nn_set_dense_layer(Model *m, int idx, int in_features, int out_features,
                       const float *weights, const float *biases,
                       Activation activation) {
  if (!m || idx < 0 || idx >= m->num_layers)
    return -1;
  DenseLayer *L = &m->layers[idx];
  L->in_features = in_features;
  L->out_features = out_features;
  size_t wlen = (size_t)in_features * out_features;
  L->weights = malloc(sizeof(float) * wlen);
  if (!L->weights)
    return -1;
  memcpy(L->weights, weights, sizeof(float) * wlen);
  L->biases = malloc(sizeof(float) * out_features);
  if (!L->biases)
    return -1;
  memcpy(L->biases, biases, sizeof(float) * out_features);
  L->act = activation;
  return 0;
}

static inline float apply_activation(float x, Activation act) {
  switch (act) {
  case ACT_RELU:
    return x > 0.f ? x : 0.f;
  case ACT_SIGMOID:
    return 1.f / (1.f + expf(-x));
  case ACT_TANH:
    return tanhf(x);
  default:
    return x;
  }
}

float *nn_run(Model *m, const float *input, int input_size, int *out_size) {
  if (!m || !input || !out_size)
    return NULL;
  int curr_size = input_size;
  float *prev = malloc(sizeof(float) * curr_size);
  if (!prev)
    return NULL;
  memcpy(prev, input, sizeof(float) * curr_size);

  for (int li = 0; li < m->num_layers; li++) {
    DenseLayer *L = &m->layers[li];
    if (L->in_features != curr_size) {
      free(prev);
      return NULL;
    }
    int outf = L->out_features;
    float *curr = malloc(sizeof(float) * outf);
    if (!curr) {
      free(prev);
      return NULL;
    }

    // linear
    for (int i = 0; i < outf; i++) {
      float sum = L->biases[i];
      float *wrow = L->weights + (size_t)i * L->in_features;
      for (int j = 0; j < L->in_features; j++)
        sum += wrow[j] * prev[j];
      curr[i] = sum;
    }
    // activation
    if (L->act == ACT_SOFTMAX) {
      float maxv = curr[0];
      for (int i = 1; i < outf; i++)
        if (curr[i] > maxv)
          maxv = curr[i];
      float s = 0.f;
      for (int i = 0; i < outf; i++) {
        curr[i] = expf(curr[i] - maxv);
        s += curr[i];
      }
      for (int i = 0; i < outf; i++)
        curr[i] /= s;
    } else {
      for (int i = 0; i < outf; i++)
        curr[i] = apply_activation(curr[i], L->act);
    }

    free(prev);
    prev = curr;
    curr_size = outf;
  }

  *out_size = curr_size;
  return prev;
}

void nn_free_model(Model *m) {
  if (!m)
    return;
  for (int i = 0; i < m->num_layers; i++) {
    free(m->layers[i].weights);
    free(m->layers[i].biases);
  }
  free(m->layers);
  free(m);
}

void nn_free_buffer(float *buffer) { free(buffer); }
