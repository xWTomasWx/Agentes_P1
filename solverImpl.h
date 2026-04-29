#pragma once
#include "solver.h"
#include <cmath>
#include <chrono>
#include <algorithm>
#include <vector>
#include <deque>
#include <iostream>

const double TIMEOUT_SEC = 30.0;

// ==========================================
// HELPER: Factor de ramificación efectivo b*
// ==========================================
static double calcBstar(std::size_t N, int d) {
    if (d <= 0 || N <= 1) return 0.0;
    if (d == 1) return static_cast<double>(N - 1);
    double lo = 1.0, hi = static_cast<double>(N);
    for (int iter = 0; iter < 200; iter++) {
        double mid = (lo + hi) / 2.0;
        double sum = 0.0, pw = 1.0;
        for (int i = 0; i <= d; i++) { sum += pw; pw *= mid; }
        if (sum < static_cast<double>(N)) lo = mid;
        else                              hi = mid;
    }
    return (lo + hi) / 2.0;
}

// ==========================================
// HELPER: Reconstrucción del camino
// ==========================================
template<int N>
int reconstruct(const std::unordered_map<P<N>,P<N>> &path, const P<N> &_node){
    std::deque<P<N>> nodes;
    auto node = _node;
    nodes.push_front(node);
    while(true){
        auto parent = path.find(node);
        if(parent == path.end()) break;
        nodes.push_front(parent->second);
        node = parent->second;
    }
    int len = static_cast<int>(nodes.size()) - 1;
    return len;
}

// ==========================================
// BFS (Breadth-First Search)
// ==========================================
template<int N>
void BFS(const P<N>& root) {
    auto start = std::chrono::high_resolution_clock::now();
    std::unordered_set<P<N>>  visited;
    std::queue<P<N>>           Q;
    std::unordered_map<P<N>,P<N>> path;

    std::size_t maxFrontier    = 0;
    std::size_t nodesGenerated = 1;   
    std::size_t nodesExpanded  = 0;

    Q.push(root);
    visited.insert(root);
    maxFrontier = std::max(maxFrontier, Q.size());

    while(!Q.empty()){
        // Verificación de Timeout
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(now - start).count() > TIMEOUT_SEC) {
            std::cout << "BFS," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
            return;
        }

        P<N> node = Q.front();
        Q.pop();
        nodesExpanded++;

        if(node.isDone()){
            auto end = std::chrono::high_resolution_clock::now();
            int solLen = reconstruct(path, node);
            double bstar = calcBstar(nodesGenerated, solLen);
            double tiempo = (end-start).count()/1000000.0;
            std::cout << "BFS," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << "," << bstar << "," << solLen << "," << tiempo << "\n";
            return;
        }

        std::vector<DIR> children;
        node.getCandidates(children);
        for(auto dir : children){
            P<N> child(node);
            child.move(dir);
            if(visited.find(child) != visited.end()) continue;
            Q.push(child);
            visited.insert(child);
            path[child] = node;
            nodesGenerated++;
            maxFrontier = std::max(maxFrontier, Q.size());
        }

        if(nodesGenerated > NODELIMIT){
            std::cout << "BFS," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
            return;
        }
    }
    std::cout << "BFS," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
}

// ==========================================
// Greedy Best-First Search
// ==========================================
template<int N>
void greedy(const P<N>& root){
    struct CompH {
        bool operator()(const P<N>&a, const P<N>&b) const { return a.H() > b.H(); }
    };
    auto start = std::chrono::high_resolution_clock::now();
    std::unordered_set<P<N>>  visited;
    std::priority_queue<P<N>, std::vector<P<N>>, CompH> Q;
    std::unordered_map<P<N>,P<N>> path;

    std::size_t maxFrontier    = 0;
    std::size_t nodesGenerated = 1;
    std::size_t nodesExpanded  = 0;

    Q.push(root);
    visited.insert(root);
    maxFrontier = std::max(maxFrontier, Q.size());

    while(!Q.empty()){
        // Verificación de Timeout
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(now - start).count() > TIMEOUT_SEC) {
            std::cout << "Greedy," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
            return;
        }

        P<N> node = Q.top();
        Q.pop();
        nodesExpanded++;

        if(node.isDone()){
            auto end = std::chrono::high_resolution_clock::now();
            int solLen = reconstruct(path, node);
            double bstar = calcBstar(nodesGenerated, solLen);
            double tiempo = (end-start).count()/1000000.0;
            std::cout << "Greedy," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << "," << bstar << "," << solLen << "," << tiempo << "\n";
            return;
        }

        std::vector<DIR> children;
        node.getCandidates(children);
        for(auto dir : children){
            P<N> child(node);
            child.move(dir);
            if(visited.find(child) != visited.end()) continue;
            Q.push(child);
            visited.insert(child);
            path[child] = node;
            nodesGenerated++;
            maxFrontier = std::max(maxFrontier, Q.size());
        }

        if(nodesGenerated > NODELIMIT){
            std::cout << "Greedy," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
            return;
        }
    }
    std::cout << "Greedy," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
}

// ==========================================
// Estructura auxiliar para A* y SMA*
// ==========================================
template<int N>
struct AStarNode {
    P<N> state;
    int  g;
    int  f() const { return g + state.H2(); }
    bool operator>(const AStarNode& o) const { return f() > o.f(); }
};

// ==========================================
// A* (A-Star) con h2 = Manhattan
// ==========================================
template<int N>
void AStar(const P<N>& root) {
    auto start = std::chrono::high_resolution_clock::now();
    std::priority_queue<AStarNode<N>, std::vector<AStarNode<N>>, std::greater<AStarNode<N>>> Q;
    std::unordered_map<P<N>, int>    g_score;
    std::unordered_map<P<N>, P<N>>   path;

    std::size_t maxFrontier    = 0;
    std::size_t nodesGenerated = 1;
    std::size_t nodesExpanded  = 0;

    Q.push({root, 0});
    g_score[root] = 0;
    maxFrontier = std::max(maxFrontier, Q.size());

    while(!Q.empty()){
        // Verificación de Timeout
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(now - start).count() > TIMEOUT_SEC) {
            std::cout << "AStar," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
            return;
        }

        AStarNode<N> current = Q.top();
        Q.pop();

        if(current.g > g_score[current.state]) continue;
        nodesExpanded++;

        if(current.state.isDone()){
            auto end = std::chrono::high_resolution_clock::now();
            int solLen = reconstruct(path, current.state);
            double bstar = calcBstar(nodesGenerated, solLen);
            double tiempo = (end-start).count()/1000000.0;
            std::cout << "AStar," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << "," << bstar << "," << solLen << "," << tiempo << "\n";
            return;
        }

        std::vector<DIR> children;
        current.state.getCandidates(children);
        for(auto dir : children){
            P<N> child(current.state);
            child.move(dir);
            int tentative_g = current.g + 1;

            if(g_score.find(child) == g_score.end() || tentative_g < g_score[child]){
                g_score[child] = tentative_g;
                path[child]    = current.state;
                Q.push({child, tentative_g});
                nodesGenerated++;
                maxFrontier = std::max(maxFrontier, Q.size());
            }
        }

        if(nodesGenerated > NODELIMIT){
            std::cout << "AStar," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
            return;
        }
    }
    std::cout << "AStar," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
}

// ==========================================
// IDA* — función recursiva auxiliar
// ==========================================
template<int N>
int searchIDA(const P<N>& node, int g, int bound,
              std::unordered_map<P<N>,P<N>>& path,
              std::unordered_set<P<N>>& path_set,
              std::size_t& nodesGenerated,
              std::size_t& nodesExpanded,
              int& maxDepth,
              const std::chrono::time_point<std::chrono::high_resolution_clock>& start)
{
    // Verificación de Timeout (Retorna -2 si se acabó el tiempo)
    auto now = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration<double>(now - start).count() > TIMEOUT_SEC) return -2;

    int f = g + node.H2();
    if(f > bound) return f;
    if(node.isDone()) return -1;

    nodesExpanded++;
    if(g > maxDepth) maxDepth = g;

    int min_val = static_cast<int>(1e9);
    std::vector<DIR> children;
    node.getCandidates(children);

    for(auto dir : children){
        P<N> child(node);
        child.move(dir);

        if(path_set.find(child) == path_set.end()){
            path_set.insert(child);
            path[child] = node;
            nodesGenerated++;

            int t = searchIDA(child, g + 1, bound, path, path_set, nodesGenerated, nodesExpanded, maxDepth, start);
            if(t == -1 || t == -2) return t; // Propagar éxito o timeout
            if(t < min_val) min_val = t;

            path_set.erase(child); 
        }
    }
    return min_val;
}

// ==========================================
// IDA* (Iterative-Deepening A*)
// ==========================================
template<int N>
void IDAStar(const P<N>& root) {
    auto start = std::chrono::high_resolution_clock::now();
    int   bound          = root.H2();
    std::size_t nodesGenerated = 1;
    std::size_t nodesExpanded  = 0;
    int   maxDepth       = 0;   

    std::unordered_map<P<N>,P<N>> path;
    std::unordered_set<P<N>>      path_set;
    path_set.insert(root);

    while(true){
        int t = searchIDA(root, 0, bound, path, path_set, nodesGenerated, nodesExpanded, maxDepth, start);

        if(t == -1){ // Solución encontrada
            auto end = std::chrono::high_resolution_clock::now();
            P<N> target;
            bool found = false;
            for(const auto& pair : path){
                if(pair.first.isDone()){ target = pair.first; found = true; break; }
            }
            int solLen = 0;
            if(found) solLen = reconstruct(path, target);
            double bstar = calcBstar(nodesGenerated, solLen);
            double tiempo = (end-start).count()/1000000.0;
            std::cout << "IDAStar," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxDepth << "," << bstar << "," << solLen << "," << tiempo << "\n";
            return;
        }
        
        // Timeout (-2) o Límite de Nodos/No encontrado
        if(t == -2 || t >= static_cast<int>(1e9) || nodesGenerated > NODELIMIT){
            std::cout << "IDAStar," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxDepth << ",-1,-1,-1\n";
            return;
        }
        bound = t;
    }
}

// ==========================================
// Beam Search
// ==========================================
template<int N>
void BeamSearch(const P<N>& root, int beam_width) {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<P<N>>           current_level;
    std::unordered_set<P<N>>    visited;
    std::unordered_map<P<N>,P<N>> path;

    std::size_t maxFrontier    = 0;
    std::size_t nodesGenerated = 1;
    std::size_t nodesExpanded  = 0;

    current_level.push_back(root);
    visited.insert(root);
    maxFrontier = std::max(maxFrontier, current_level.size());

    while(!current_level.empty()){
        // Verificación de Timeout
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(now - start).count() > TIMEOUT_SEC) {
            std::cout << "BeamSearch," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
            return;
        }

        std::vector<P<N>> next_level;

        for(const auto& node : current_level){
            nodesExpanded++;

            if(node.isDone()){
                auto end = std::chrono::high_resolution_clock::now();
                int solLen = reconstruct(path, node);
                double bstar = calcBstar(nodesGenerated, solLen);
                double tiempo = (end-start).count()/1000000.0;
                std::cout << "BeamSearch," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << "," << bstar << "," << solLen << "," << tiempo << "\n";
                return;
            }

            std::vector<DIR> children;
            node.getCandidates(children);
            for(auto dir : children){
                P<N> child(node);
                child.move(dir);
                if(visited.find(child) == visited.end()){
                    visited.insert(child);
                    path[child] = node;
                    next_level.push_back(child);
                    nodesGenerated++;
                }
            }
        }

        if(nodesGenerated > NODELIMIT){
             std::cout << "BeamSearch," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
            return;
        }

        std::sort(next_level.begin(), next_level.end(), [](const P<N>& a, const P<N>& b){
            return a.H2() < b.H2();
        });
        if(next_level.size() > static_cast<std::size_t>(beam_width))
            next_level.resize(beam_width);

        maxFrontier = std::max(maxFrontier, next_level.size());
        current_level = std::move(next_level);
    }
    std::cout << "BeamSearch," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
}

// ==========================================
// RBFS — función recursiva auxiliar
// ==========================================
template<int N>
std::pair<P<N>, int> RBFS_Recursive(
        const P<N>& node, const P<N>* parent,
        int g, int f_node, int f_limit,
        std::unordered_map<P<N>,P<N>>& path,
        std::size_t& nodesGenerated,
        std::size_t& nodesExpanded,
        std::size_t& maxFrontier,
        const std::chrono::time_point<std::chrono::high_resolution_clock>& start)
{
    // Verificación de Timeout
    auto now = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration<double>(now - start).count() > TIMEOUT_SEC) return {node, -2};

    if(node.isDone()) return {node, -1};

    nodesExpanded++;
    std::vector<DIR> candidates;
    node.getCandidates(candidates);

    std::vector<std::pair<P<N>,int>> successors;
    for(auto dir : candidates){
        P<N> child(node);
        child.move(dir);
        if(parent != nullptr && child == *parent) continue;   
        nodesGenerated++;
        int child_f = std::max(g + 1 + child.H2(), f_node);  
        successors.push_back({child, child_f});
    }
    maxFrontier = std::max(maxFrontier, successors.size());

    if(successors.empty()) return {node, static_cast<int>(1e9)};

    while(true){
        if(nodesGenerated > NODELIMIT) return {node, static_cast<int>(1e9)};

        std::sort(successors.begin(), successors.end(),
                  [](const auto& a, const auto& b){ return a.second < b.second; });

        auto& best = successors[0];
        if(best.second > f_limit) return {node, best.second};

        int alternative_f = (successors.size() > 1) ? successors[1].second : static_cast<int>(1e9);

        auto result = RBFS_Recursive(best.first, &node, g + 1, best.second,
                                     std::min(f_limit, alternative_f),
                                     path, nodesGenerated, nodesExpanded, maxFrontier, start);
        
        if(result.second == -1 || result.second == -2){
            if (result.second == -1) path[best.first] = node; // Solo guardar ruta si es éxito
            return result;
        }
        best.second = result.second;
    }
}

// ==========================================
// RBFS (Recursive Best-First Search)
// ==========================================
template<int N>
void RBFS(const P<N>& root) {
    auto start = std::chrono::high_resolution_clock::now();
    std::unordered_map<P<N>,P<N>> path;
    std::size_t nodesGenerated = 1;
    std::size_t nodesExpanded  = 0;
    std::size_t maxFrontier    = 0;

    int f_root = root.H2();
    auto result = RBFS_Recursive<N>(root, nullptr, 0, f_root,
                                    static_cast<int>(1e9),
                                    path, nodesGenerated, nodesExpanded, maxFrontier, start);

    if(result.second == -1){ // Éxito
        auto end = std::chrono::high_resolution_clock::now();
        int solLen = reconstruct(path, result.first);
        double bstar = calcBstar(nodesGenerated, solLen);
        double tiempo = (end-start).count()/1000000.0;
        std::cout << "RBFS," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << "," << bstar << "," << solLen << "," << tiempo << "\n";
    } else { // Timeout (-2) o Nodos límite
        std::cout << "RBFS," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
    }
}

// ==========================================
// SMA* (Simplified Memory-bounded A*)
// ==========================================
template<int N>
void SMAStar(const P<N>& root, std::size_t memory_limit) {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<AStarNode<N>>       open_list;
    std::unordered_map<P<N>, int>   g_score;
    std::unordered_map<P<N>, P<N>>  path;

    std::size_t maxFrontier    = 0;
    std::size_t nodesGenerated = 1;
    std::size_t nodesExpanded  = 0;

    open_list.push_back({root, 0});
    g_score[root] = 0;
    maxFrontier = std::max(maxFrontier, open_list.size());

    while(!open_list.empty()){
        // Verificación de Timeout
        auto now = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(now - start).count() > TIMEOUT_SEC) {
            std::cout << "SMAStar," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
            return;
        }

        std::sort(open_list.begin(), open_list.end(), std::greater<AStarNode<N>>());
        AStarNode<N> current = open_list.back();
        open_list.pop_back();
        nodesExpanded++;

        if(current.state.isDone()){
            auto end = std::chrono::high_resolution_clock::now();
            int solLen = reconstruct(path, current.state);
            double bstar = calcBstar(nodesGenerated, solLen);
            double tiempo = (end-start).count()/1000000.0;
            std::cout << "SMAStar," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << "," << bstar << "," << solLen << "," << tiempo << "\n";
            return;
        }

        std::vector<DIR> children;
        current.state.getCandidates(children);
        for(auto dir : children){
            P<N> child(current.state);
            child.move(dir);
            int tentative_g = current.g + 1;

            if(g_score.find(child) == g_score.end() || tentative_g < g_score[child]){
                g_score[child] = tentative_g;
                path[child]    = current.state;
                open_list.push_back({child, tentative_g});
                nodesGenerated++;
            }
        }

        if(open_list.size() > memory_limit){
            std::sort(open_list.begin(), open_list.end(), std::greater<AStarNode<N>>());
            open_list.erase(open_list.begin(),
                            open_list.begin() + (open_list.size() - memory_limit));
        }

        maxFrontier = std::max(maxFrontier, open_list.size());

        if(nodesGenerated > NODELIMIT){
            std::cout << "SMAStar," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
            return;
        }
    }
    std::cout << "SMAStar," << N << "," << shuffleIterations << "," << nodesGenerated << "," << nodesExpanded << "," << maxFrontier << ",-1,-1,-1\n";
}