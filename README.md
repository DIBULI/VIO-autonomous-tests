# SLAM-VIO-autonomous-tests

This repo has been tested with the docker based on Ubuntu 20.04_x86.

# How to start the autonomous tests

## VIO related autonomous test

### Prepare the dataset

Use the VIOs/scripts/prepare-dataset.sh to download the required dataset

```
bash VIOs/scripts/prepare-dataset.sh
```

### Initialize the workspace using Docker

```
cd VIOs && docker build .
```

### Run all the tests

All the results will be output into the `tests-result` directory.

#### Run specific tests

```
// TODO
```