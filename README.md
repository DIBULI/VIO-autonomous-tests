# SLAM-VIO-autonomous-tests

This repo has been tested with the docker based on Ubuntu 20.04_x86.

# How to start the autonomous tests

## VIO related autonomous test

### Prepare the dataset

Use the `scripts/prepare-dataset.sh` to download the required dataset
You can modify the variable `DATA_PATH` in the shell script where the datasets are gonna be stored
```
bash /scripts/prepare-dataset.sh
```

### Initialize the workspace using Docker

```
bash build.sh
```

### Run all the tests

All the results will be output into the `tests-result` directory.

#### Run specific tests

You can modify the variable `DATA_PATH` in the shell script to match your dataset folder location.
You can also modify the variable `TEST_RESULT_FOLDER` to define where to store all the results
```
bash run.sh
```

The VIOs result will be stored under the folder defined by `TEST_RESULT_FOLDER`.

