We suggest having two seprate virtual environments for `DOPE` and `Detectron 2`.

We assume that `CUDA 10.1` is adopted.

### DOPE

Just install the provided [requirements](/docs/dope_requirements.txt)

### Detectron 2

Install the provided [requirements](/docs/detectron2_requirements.txt).

Then proceed with `detectron 2` installation as follows:

```
python -m pip install detectron2 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu101/torch1.6/index.html
```

