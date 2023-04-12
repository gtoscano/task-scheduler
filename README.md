# Project Title: Task Scheduler for Multiprocessor Environments

Task Scheduler is a high-performance C++ implementation for scheduling tasks on a multiprocessor environment. The algorithm is based on the Longest Processing Time (LPT) approach, with a modification that uses binary search to find the optimal number of processors to use. The tasks and their dependencies are provided as input in a JSON file, and the output is the optimal number of processors and the scheduling of each task on each processor.

## Table of Contents

- [Features](#features)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Input JSON Format](#input-json-format)
- [Output](#output)
- [Contributing](#contributing)
- [License](#license)

## Features

- Efficient scheduling of tasks with dependencies on a multiprocessor environment.
- Utilizes binary search to find the optimal number of processors.
- High-performance C++ implementation.
- Easy to use JSON input format for tasks and dependencies.

## Dependencies

- C++11 or later.
- [nlohmann/json](https://github.com/nlohmann/json) - for parsing JSON input files.

## Installation

1. Clone the repository:

```bash
git clone https://github.com/gtoscano/task-scheduler.git
```

2. Compile the project:

```bash

mkdir build
cd build
cmake ..
make
```

3. Compile the project for debug:

```bash

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```
## Usage

After compiling the project, use the following command to run the task scheduler:

```bash

./task-scheduler input.json
```

Replace input.json with the path to your JSON file containing the tasks and dependencies.

## Input JSON Format

The input JSON file should contain two main keys: "tasks" and "dependencies". The "tasks" key should have an object with task IDs as keys and their respective execution times as values. The "dependencies" key should have an array of arrays, where each inner array contains two elements: the first is the parent task ID, and the second is the child task ID. The input graph must be a directed acyclic graph (DAG) and a tree (i.e., each node has only one single child).

Example:

```bash
json

{
    "tasks": {
        "1": 10,
        "2": 20,
        "3": 30,
        "4": 40,
        "5": 50,
        "6": 60,
        "7": 70,
        "8": 80,
        "9": 90
    },
    "dependencies": [
        [1, 2],
        [2, 6],
        [4, 5],
        [5, 6]
    ]
}
```

## Output

The output will display the optimal number of processors and the scheduling of each task on each processor. The output format is a list of tasks with their corresponding processor number.

Example:

```bash
Best number of processors: 3
Scheduling:
Processor 1: Task 1 -> Task 2 -> Task 6
Processor 2: Task 3
Processor 3: Task 4 -> Task 5
```

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.
License

## MIT License

[MIT License](https://choosealicense.com/licenses/mit/)

