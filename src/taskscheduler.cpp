#include "execqueue.h"
#include "myqueue.h"


#include <nlohmann/json.hpp>
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <list>
#include <limits>
#include <queue>
#include <set>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>



std::vector<int> find_genesis_nodes(const std::unordered_map<int, std::list<int>>& dag) {
    std::unordered_set<int> nodes_with_incoming_edges;

    // Iterate through the DAG and record nodes with incoming edges
    for (const auto& node : dag) {
        for (int to_task : node.second) {
            nodes_with_incoming_edges.insert(to_task);
        }
    }

    // Find nodes that don't have incoming edges
    std::vector<int> first_nodes;
    for (const auto& node : dag) {
        if (nodes_with_incoming_edges.find(node.first) == nodes_with_incoming_edges.end()) {
            first_nodes.push_back(node.first);
        }
    }

    return first_nodes;
}

std::unordered_map<int, std::list<int>> reverse_dag(const std::unordered_map<int, std::list<int>>& dag) {
    std::unordered_map<int, std::list<int>> reversed_dag;

    for (const auto& node : dag) {
        int from_task = node.first;
        for (int to_task : node.second) {
            if (reversed_dag.find(to_task) == reversed_dag.end()) {
                reversed_dag[to_task] = std::list<int>();
            }
            reversed_dag[to_task].push_back(from_task);
        }
    }

    return reversed_dag;
}

std::vector<int> topological_sort(const std::unordered_map<int, std::list<int>>& dag) {
    std::unordered_map<int, int> in_degree;
    for (const auto& [node, adj_list] : dag) {
        if (!in_degree.count(node)) {
            in_degree[node] = 0;
        }
        for (int neighbor : adj_list) {
            ++in_degree[neighbor];
        }
    }

    std::stack<int> zero_degree_nodes;
    for (const auto& [node, degree] : in_degree) {
        if (degree == 0) {
            zero_degree_nodes.push(node);
        }
    }

    std::vector<int> sorted_tasks;
    while (!zero_degree_nodes.empty()) {
        int current = zero_degree_nodes.top();
        zero_degree_nodes.pop();
        sorted_tasks.push_back(current);

        if (dag.count(current)) {
            for (int neighbor : dag.at(current)) {
                if (--in_degree[neighbor] == 0) {
                    zero_degree_nodes.push(neighbor);
                }
            }
        }
    }

    return sorted_tasks;
}

std::unordered_map<int, std::list<int>> build_dag(
         const std::vector<std::pair<int, int>>& dependencies
        ) {
    std::unordered_map<int, std::list<int>> dag;
    for (const auto& [u, v] : dependencies) {
        dag[u].push_back(v);
    }
    return dag;
}

std::unordered_map<int, int>
compute_cost(
        const std::unordered_map<int, std::list<int>>& rdag, 
        const std::unordered_map<int, int>& tasks,
        const std::vector<int> roots
        ) {

    std::unordered_map<int, int> acc_costs;

    std::vector<int> queue(roots);
    for( const auto& element : roots) {
        acc_costs[element] = tasks.at(element);
    }
    while (! queue.empty()) {
        auto element = queue.front();
        queue.erase(queue.begin());
        if (rdag.contains(element)) {
            for(const auto& node: rdag.at(element)) {
                if (acc_costs.contains(node)) {
                    std::cerr<<"Error, this node has been previously stored\n";
                    exit(-1);
                }
                acc_costs[node] = acc_costs[element] + tasks.at(node);
                queue.push_back(node);
            }
        }

    }
    return acc_costs;
}
void print_priority(std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>> q) {
    // NB: q is passed by value because there is no way to traverse
    // priority_queue's content without erasing the queue.
    std::cout<<"============\n";
    for (; !q.empty(); ){
        auto [node_acc_cost, node] = q.top();
        std::cout << node << " ";
        q.pop();
    }

    std::cout << '\n';
}



void dequeue(
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>>& tasks_queue,
        const std::unordered_map<int, std::list<int>>& dag,
        const std::unordered_map<int, std::list<int>>& rev_dag,
        std::set<int>& already_executed,
        const std::unordered_map<int, int>& costs,
        int node
        ) {

            already_executed.insert(node);
            if (dag.find(node) != dag.end()) {
                for (const auto& next_node : dag.at(node)) {
                    auto dependent_nodes = rev_dag.at(next_node);
                    bool flag = true;
                    for(const auto& dependent : dependent_nodes) {
                        auto found_iter = already_executed.find(dependent);
                        if (found_iter == already_executed.end()) { 
                            flag = false;
                            break;
                        }
                    }
                    if (flag == true) {
                        auto node_cost = costs.at(next_node);
                        tasks_queue.push({node_cost, next_node});
                    }
                }
            }
}


ExecQueue
lpt_algorithm(
        const std::unordered_map<int, int>& tasks,
        const std::vector<std::pair<int, int>>& dependencies,
        const std::unordered_map<int, std::list<int>>& dag,
        const std::vector<int>& roots,
        const std::unordered_map<int, std::list<int>>& rev_dag,
        const std::vector<int>& rev_roots,
        const std::unordered_map<int, int>& costs,
        int m) {
    
    ExecQueue exec_queue(m);
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>> tasks_queue;
    std::set<int> already_executed;
    

    for (const auto& node: roots) {
        auto node_cost = costs.at(node);
        tasks_queue.push({node_cost, node});
    }

    int total_time = 0;


    while(!tasks_queue.empty() || !exec_queue.is_empty()){
        auto results = exec_queue.dequeue();
        for (const auto& node: results) {
            dequeue( tasks_queue, dag, rev_dag, already_executed, costs, node);
        }
        //enqueue
        auto counter = exec_queue.count_open_queues();
        counter = (counter<tasks_queue.size())?counter:tasks_queue.size();
        for(int i=0; i< counter; ++i) {
            auto [node_acc_cost, node] = tasks_queue.top();
            auto node_cost = tasks.at(node);
            tasks_queue.pop();
            exec_queue.enqueue(node, node_cost);
        }
        
    } 

    return exec_queue;
}



int find_best_num_cpus(
        const std::unordered_map<int, int>& tasks,
        const std::vector<std::pair<int, int>>& dependencies,
        const std::unordered_map<int, std::list<int>>& dag,
        const std::vector<int>& roots,
        const std::unordered_map<int, std::list<int>>& rev_dag,
        const std::vector<int>& rev_roots,
        const std::unordered_map<int, int>& costs,
        int min_cpus, int max_cpus) {
    int best_cpus = max_cpus;
    auto nqueues = tasks.size();
    int min_makespan = std::numeric_limits<int>::max();

    while (min_cpus <= max_cpus) {
        int mid_cpus = min_cpus + (max_cpus - min_cpus) / 2;

        auto exec_queue = lpt_algorithm(tasks,dependencies, dag, roots, rev_dag, rev_roots, costs, mid_cpus);
        auto makespan = exec_queue.get_total_time();
        auto queue_time = exec_queue.get_in_time();
        int queue_size = queue_time.size();
        if ((makespan == min_makespan && queue_size < nqueues) ||  (makespan < min_makespan)) {
            min_makespan = makespan;
            best_cpus = mid_cpus;
            max_cpus = mid_cpus - 1;
            nqueues = queue_size;
        } else {
            min_cpus = mid_cpus + 1;
        }
    }

    return best_cpus;
}

int find_makespan(
        const std::unordered_map<int, int>& tasks,
        const std::vector<std::pair<int, int>>& dependencies,
        const std::unordered_map<int, std::list<int>>& dag,
        const std::vector<int>& roots,
        const std::unordered_map<int, std::list<int>>& rev_dag,
        const std::vector<int>& rev_roots,
        const std::unordered_map<int, int>& costs,
        int min_cpus,
        int max_cpus,
        int desired_makespan) {
    int best_cpus = max_cpus;
    auto nqueues = tasks.size();
    int min_makespan = std::numeric_limits<int>::max();

    while (min_cpus <= max_cpus) {
        int mid_cpus = min_cpus + (max_cpus - min_cpus) / 2;

        auto exec_queue = lpt_algorithm(tasks,dependencies, dag, roots, rev_dag, rev_roots, costs, mid_cpus);
        auto makespan = exec_queue.get_total_time();
        auto queue_time = exec_queue.get_in_time();
        int queue_size = queue_time.size();
        int delta_makespan = std::abs(desired_makespan-makespan);
        int delta_min_makespan = std::abs(desired_makespan-min_makespan);
        if ((delta_makespan == delta_min_makespan && queue_size < nqueues) ||  (delta_makespan < delta_min_makespan)) {
            min_makespan = makespan;
            best_cpus = mid_cpus;
            max_cpus = mid_cpus - 1;
            nqueues = queue_size;
        } else {
            min_cpus = mid_cpus + 1;
        }
    }

    return best_cpus;
}

std::unordered_map<int, int> read_tasks(const nlohmann::json& json_data) {
    std::unordered_map<int, int> tasks;
    for (const auto& [key, value] : json_data.items()) {
        tasks[std::stoi(key)] = value.get<int>();
    }
    return tasks;
}

std::vector<std::pair<int, int>> read_dependencies(const nlohmann::json& json_data) {
    std::vector<std::pair<int, int>> dependencies;
    for (const auto& dep : json_data) {
        dependencies.push_back({dep[0].get<int>(), dep[1].get<int>()});
    }
    return dependencies;
}



int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: " << argv[0] << " <input.json> [<max_processors>]\n";
        return 1;
    }

    std::string input_json_file = argv[1];
    if (!std::filesystem::exists(input_json_file)) {
        std::cout << "The file " << input_json_file << " does not exist.\n";
        return 1;
    }

    std::ifstream input_file(input_json_file);
    nlohmann::json json_data;
    input_file >> json_data;

    std::unordered_map<int, int> tasks = read_tasks(json_data["tasks"]);
    std::vector<std::pair<int, int>> dependencies = read_dependencies(json_data["dependencies"]);

    int min_cpus = 1;
    int max_cpus = (argc == 3) ? std::stoi(argv[2]) : tasks.size(); 
    auto dag = build_dag(dependencies);
    auto roots = find_genesis_nodes(dag);
    auto rev_dag = reverse_dag(dag);
    auto rev_roots = find_genesis_nodes(rev_dag);
    auto costs = compute_cost(rev_dag, tasks, rev_roots);

    int best_num_cpus = find_best_num_cpus(tasks, dependencies, dag, roots, rev_dag, rev_roots, costs, min_cpus,  max_cpus);
    std::cout << "Best number of processors: " << best_num_cpus << std::endl;
    auto exec_queue = lpt_algorithm(tasks, dependencies, dag, roots, rev_dag, rev_roots, costs, best_num_cpus);
    exec_queue.print();
    /*
    int makespan = 180;
    best_num_cpus = find_makespan(tasks, dependencies, dag, roots, rev_dag, rev_roots, costs, min_cpus, max_cpus, makespan);
    std::cout << "Best number of CPUs: " << best_num_cpus << std::endl;
    auto exec_queue2 = lpt_algorithm(tasks, dependencies, dag, roots, rev_dag, rev_roots, costs, best_num_cpus);
    exec_queue2.print();
    std::cout<<best_num_cpus<<std::endl;
    */
    return 0;
}

