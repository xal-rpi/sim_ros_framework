import torch


def main():
    print("PyTorch version:", torch.__version__)
    print("CUDA available:", torch.cuda.is_available())
    if torch.cuda.is_available():
        print("CUDA version:", torch.version.cuda)
        print("cuDNN version:", torch.backends.cudnn.version())
        device = torch.device("cuda")
        # simple tensor operation on GPU
        x = torch.rand(3, 3, device=device)
        y = x * 2
        print("Tensor on CUDA device:\n", y)
    else:
        print("No CUDA device detected")


if __name__ == "__main__":
    main()
