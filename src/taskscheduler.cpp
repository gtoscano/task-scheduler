#include "taskscheduler.h"
#include "execqueue.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <list>
#include <limits>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>


TaskScheduler::TaskScheduler(const std::string& config_file) {
    load_config_file(config_file);
}

bool TaskScheduler::load_config_file(const std::string& config_file){
    std::ifstream input_file(config_file);
    nlohmann::json json_data;
    input_file >> json_data;

    tasks_ = read_tasks(json_data["tasks"]);
    dependencies_ = read_dependencies(json_data["dependencies"]);
    build_dag();
    roots_ = find_genesis_nodes(dag_);
    rev_dag_ = reverse_dag(dag_);
    rev_roots_ = find_genesis_nodes(rev_dag_);
    costs_ = compute_cost();
    return true;
}

std::unordered_map<int, int> TaskScheduler::read_tasks(const nlohmann::json& json_data){
    std::unordered_map<int, int>  tasks;
    for (const auto& [key, value] : json_data.items()) {
        tasks[std::stoi(key)] = value.get<int>();
    }
    return tasks;
}

std::vector<std::pair<int, int>> TaskScheduler::read_dependencies(const nlohmann::json& json_data) {
    std::vector<std::pair<int, int>>  dependencies;
    for (const auto& dep : json_data) {
        dependencies.emplace_back(dep[0].get<int>(), dep[1].get<int>());
    }
    return dependencies;
}

void TaskScheduler::build_dag() {
    for (const auto& [u, v] : dependencies_) {
        dag_[u].emplace_back(v);
    }
}

std::vector<int> TaskScheduler::find_genesis_nodes(const std::unordered_map<int, std::list<int>>& dag) {
    std::unordered_set<int> nodes_with_incoming_edges;

    // Iterate through the DAG and record nodes with incoming edges
    for (const auto& [key, value] : dag) {
        for (int to_task : value) {
            nodes_with_incoming_edges.insert(to_task);
        }
    }

    // Find nodes that don't have incoming edges
    std::vector<int> first_nodes;
    for (const auto& [key, value] : dag) {
        if (!nodes_with_incoming_edges.contains(key)) {
            first_nodes.emplace_back(key);
        }
    }

    return first_nodes;
}


std::unordered_map<int, std::list<int>> TaskScheduler::reverse_dag(const std::unordered_map<int, std::list<int>>& dag) {
    std::unordered_map<int, std::list<int>> reversed_dag;
    for (const auto& [key, value]: dag) {
        int from_task = key;
        for (int to_task : value) {
            if (!reversed_dag.contains(to_task)) {
                reversed_dag[to_task] = std::list<int>();
            }
            reversed_dag[to_task].emplace_back(from_task);
        }
    }

    return reversed_dag;
}

std::unordered_map<int, int>
TaskScheduler::compute_cost() {
    std::unordered_map<int, int> acc_costs;

    std::vector<int> queue(rev_roots_);
    for( const auto& element : rev_roots_) {
        acc_costs[element] = tasks_.at(element);
    }

    while (! queue.empty()) {
        auto element = queue.front();
        queue.erase(queue.begin());
        if (rev_dag_.contains(element)) {
            for(const auto& node: rev_dag_.at(element)) {
                if (acc_costs.contains(node)) {
                    std::cerr<<"Error, this node has been previously stored\n";
                    exit(-1);
                }
                acc_costs[node] = acc_costs[element] + tasks_.at(node);
                queue.emplace_back(node);
            }
        }

    }
    return acc_costs;
}

void TaskScheduler::print_priority(std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>> q) const {
    // NB: q is passed by value because there is no way to traverse
    // priority_queue's content without erasing the queue.
    while( !q.empty() ){
        auto [node_acc_cost, node] = q.top();
        std::cout << node << " ";
        q.pop();
    }

    std::cout << '\n';
}



void TaskScheduler::dequeue(
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>>& tasks_queue,
        std::set<int>& already_executed,
        int node) {
    already_executed.insert(node);
    if (dag_.contains(node)) {
        for (const auto& next_node : dag_.at(node)) {
            auto dependent_nodes = rev_dag_.at(next_node);
            bool flag = true;
            for(const auto& dependent : dependent_nodes) {

                auto found_iter = already_executed.find(dependent);
                if (found_iter == already_executed.end()) {
                    flag = false;
                    break;
                }
            }
            if (flag) {
                auto node_cost = costs_.at(next_node);
                tasks_queue.emplace(node_cost, next_node);
            }
        }
    }
}


ExecQueue TaskScheduler::lpt_algorithm(int m) {
    ExecQueue exec_queue(m);
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>> tasks_queue;
    std::set<int> already_executed;

    for (const auto& node: roots_) {
        auto node_cost = costs_.at(node);
        tasks_queue.emplace(node_cost, node);
    }

    while(!tasks_queue.empty() || !exec_queue.is_empty()){
        for (auto results = exec_queue.dequeue(); const auto& node: results) {
            dequeue(tasks_queue, already_executed, node);
        }
        //enqueue
        size_t counter = exec_queue.count_open_queues();
        counter = (counter < tasks_queue.size())? counter : tasks_queue.size();
        for(int i=0; i< counter; ++i) {
            auto [node_acc_cost, node] = tasks_queue.top();
            auto node_cost = tasks_.at(node);
            tasks_queue.pop();
            exec_queue.enqueue(node, node_cost);
        }
        
    } 

    return exec_queue;
}



int TaskScheduler::find_best_num_cpus(int min_cpus, int max_cpus) {
    if(max_cpus == -10) {
        max_cpus = (int) tasks_.size();
    }
    auto best_cpus = max_cpus;

    auto nqueues = tasks_.size();
    int min_makespan = std::numeric_limits<int>::max();

    while (min_cpus <= max_cpus) {
        int mid_cpus = std::midpoint(min_cpus, max_cpus);

        auto exec_queue = lpt_algorithm(mid_cpus);
        auto makespan = exec_queue.get_total_time();
        auto queue_time = exec_queue.get_in_time();
        auto queue_size = queue_time.size();
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

int TaskScheduler::find_makespan(int min_cpus, int max_cpus, int desired_makespan) {
    int best_cpus = max_cpus;
    auto nqueues = tasks_.size();
    int min_makespan = std::numeric_limits<int>::max();

    while (min_cpus <= max_cpus) {
        int mid_cpus = std::midpoint(min_cpus, max_cpus);

        auto exec_queue = lpt_algorithm(mid_cpus);
        auto makespan = exec_queue.get_total_time();
        auto queue_time = exec_queue.get_in_time();
        auto queue_size = queue_time.size();
        auto delta_makespan = std::abs(desired_makespan-makespan);
        auto delta_min_makespan = std::abs(desired_makespan-min_makespan);
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


    int min_cpus = 1;
    int max_cpus = (argc == 3) ? std::stoi(argv[2]) : -10;
    TaskScheduler ts(input_json_file);
    int best_num_cpus = ts.find_best_num_cpus(min_cpus,  max_cpus);

    std::cout << "Best number of processors: " << best_num_cpus << std::endl;
    auto exec_queue = ts.lpt_algorithm(best_num_cpus);
    exec_queue.print();
    /*
    int makespan = 180;
    best_num_cpus = find_makespan(min_cpus, max_cpus, makespan);
    std::cout << "Best number of CPUs: " << best_num_cpus << std::endl;
    auto exec_queue2 = lpt_algorithm(best_num_cpus);
    exec_queue2.print();
    std::cout<<best_num_cpus<<std::endl;
    */
    return 0;
}

