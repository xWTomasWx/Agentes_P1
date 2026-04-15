#pragma once
#include "solver.h"

template<int N>
void reconstruct(const std::unordered_map<P<N>,P<N>> &path, const P<N> &_node){
	std::deque<P<N>> nodes;
	auto node = _node;
	nodes.push_front(node);
	while(true){
		auto parent=path.find(node);
		if(parent==path.end()) break;
		nodes.push_front(parent->second);
		node=parent->second;
	}
	std::cout<<"solution length: "<<nodes.size()-1<<std::endl;
	std::cout.flush();
}

template<int N>
void BFS(const P<N>& root) {
	std::cout<<"===========================\nRunning BFS...\n";
	auto start = std::chrono::high_resolution_clock::now();

	std::unordered_set<P<N>> visited;
	std::queue<P<N>> Q;
	std::size_t maxQ = 0;
	std::size_t uniqueUsefulNodesGenerated = 1;

	std::unordered_map<P<N>,P<N>> path;

	Q.push(root);
	maxQ = std::max(maxQ, Q.size());
	visited.insert(root);

	while(!Q.empty()){
		P<N> node=Q.front();
		Q.pop();

		if(node.isDone()){
			auto end = std::chrono::high_resolution_clock::now();
			reconstruct(path,node);
			std::cout<<"Unique Nodes Visited: "<<visited.size()<<std::endl;
			std::cout<<"Unique Useful Nodes Generated: "<<uniqueUsefulNodesGenerated<<std::endl;
			std::cout<<"Max Q: "<<maxQ<<std::endl;
			std::cout<<"FOUND in "<<(end-start).count()/1000000.0<<"ms\n";
			return;
		}

		std::vector<DIR> children;
		node.getCandidates(children);
		for(auto dir:children){
			P<N> child(node);
			child.move(dir);
			if(visited.find(child)!=visited.end()) continue;
			Q.push(child);
			maxQ = std::max(maxQ, Q.size());
			visited.insert(child);
			uniqueUsefulNodesGenerated++;
			path[child]=node;
		}

		if(uniqueUsefulNodesGenerated > NODELIMIT){
			std::cerr<<"max nodes reached, aborting\n";
			return;
		}
	}
	std::cout<<"NOT FOUND!!!!\n";
}

template<int N>
void greedy(const P<N>& root){
	std::cout<<"===========================\nRunning GreedyBeFS...\n";
	struct CompH {
		bool operator()(const P<N>&a,const P<N>&b) const {
			return a.H()>b.H();
		}
	};
	auto start = std::chrono::high_resolution_clock::now();

	std::unordered_set<P<N>> visited;
	std::priority_queue<P<N>,std::vector<P<N>>,CompH> Q;
	std::size_t maxQ = 0;
	std::size_t uniqueUsefulNodesGenerated = 1;
	std::unordered_map<P<N>,P<N>> path;

	Q.push(root);
	maxQ = std::max(maxQ, Q.size());
	visited.insert(root);

	while(!Q.empty()){
		P<N> node=Q.top();
		Q.pop();

		if(node.isDone()){
			auto end = std::chrono::high_resolution_clock::now();
			reconstruct(path,node);
			std::cout<<"Unique Nodes Visited: "<<visited.size()<<std::endl;
			std::cout<<"Unique Useful Nodes Generated: "<<uniqueUsefulNodesGenerated<<std::endl;
			std::cout<<"Max Q: "<<maxQ<<std::endl;
			std::cout<<"FOUND in "<<(end-start).count()/1000000.0<<"ms\n";
			return;
		}

		std::vector<DIR> children;
		node.getCandidates(children);
		for(auto dir:children){
			P<N> child(node);
			child.move(dir);
			if(visited.find(child)!=visited.end()) continue;
			Q.push(child);
			maxQ = std::max(maxQ, Q.size());
			visited.insert(child);
			uniqueUsefulNodesGenerated++;
			path[child]=node;
		}

		if(uniqueUsefulNodesGenerated > NODELIMIT){
			std::cerr<<"max nodes reached, aborting\n";
			return;
		}
	}
	std::cout<<"NOT FOUND!!!!\n";
}


// ==========================================
// 2.2 NUEVOS ALGORITMOS
// ==========================================

// Estructura auxiliar para A* y SMA*
template<int N>
struct AStarNode {
	P<N> state;
	int g;
	int f() const { return g + state.H2(); } // Usando h2 (Manhattan)
	bool operator>(const AStarNode& other) const {
		return f() > other.f();
	}
};

// --- A* (A-Star) ---
template<int N>
void AStar(const P<N>& root) {
	std::cout << "===========================\nRunning A* (h2)...\n";
	auto start = std::chrono::high_resolution_clock::now();

	std::priority_queue<AStarNode<N>, std::vector<AStarNode<N>>, std::greater<AStarNode<N>>> Q;
	std::unordered_map<P<N>, int> g_score;
	std::unordered_map<P<N>, P<N>> path;
	std::size_t maxQ = 0;
	std::size_t uniqueUsefulNodesGenerated = 1;

	Q.push({root, 0});
	g_score[root] = 0;
	maxQ = std::max(maxQ, Q.size());

	while(!Q.empty()) {
		AStarNode<N> current = Q.top();
		Q.pop();

		if(current.state.isDone()) {
			auto end = std::chrono::high_resolution_clock::now();
			reconstruct(path, current.state);
			std::cout << "Unique Nodes Visited/Stored: " << g_score.size() << "\n";
			std::cout << "Unique Useful Nodes Generated: " << uniqueUsefulNodesGenerated << "\n";
			std::cout << "Max Q: " << maxQ << "\n";
			std::cout << "FOUND in " << (end-start).count()/1000000.0 << "ms\n";
			return;
		}

		if (current.g > g_score[current.state]) continue; // Skip obsolete states

		std::vector<DIR> children;
		current.state.getCandidates(children);
		for(auto dir : children) {
			P<N> child(current.state);
			child.move(dir);
			int tentative_g = current.g + 1;

			if(g_score.find(child) == g_score.end() || tentative_g < g_score[child]) {
				g_score[child] = tentative_g;
				path[child] = current.state;
				Q.push({child, tentative_g});
				maxQ = std::max(maxQ, Q.size());
				uniqueUsefulNodesGenerated++;
			}
		}

		if(uniqueUsefulNodesGenerated > NODELIMIT) {
			std::cerr << "max nodes reached, aborting\n";
			return;
		}
	}
	std::cout << "NOT FOUND!!!!\n";
}

// --- IDA* Auxiliar ---
template<int N>
int searchIDA(const P<N>& node, int g, int bound, std::unordered_map<P<N>, P<N>>& path, 
              std::unordered_set<P<N>>& path_set, std::size_t& nodes_gen) {
	int f = g + node.H2();
	if (f > bound) return f;
	if (node.isDone()) return -1; // -1 indica que se encontró la meta

	int min_val = 1e9; // Infinito
	std::vector<DIR> children;
	node.getCandidates(children);

	for (auto dir : children) {
		P<N> child(node);
		child.move(dir);

		if (path_set.find(child) == path_set.end()) { // Evitar ciclos en el camino actual
			path_set.insert(child);
			path[child] = node;
			nodes_gen++;
			
			int t = searchIDA(child, g + 1, bound, path, path_set, nodes_gen);
			if (t == -1) return -1;
			if (t < min_val) min_val = t;
			
			path_set.erase(child); // Backtrack
		}
	}
	return min_val;
}

// --- IDA* (Iterative-Deepening A*) ---
template<int N>
void IDAStar(const P<N>& root) {
	std::cout << "===========================\nRunning IDA* (h2)...\n";
	auto start = std::chrono::high_resolution_clock::now();
	
	int bound = root.H2();
	std::unordered_map<P<N>, P<N>> path;
	std::unordered_set<P<N>> path_set;
	std::size_t nodes_gen = 1;

	path_set.insert(root);

	while (true) {
		int t = searchIDA(root, 0, bound, path, path_set, nodes_gen);
		if (t == -1) {
			auto end = std::chrono::high_resolution_clock::now();
			std::cout << "Nodes Generated (Total across all iterations): " << nodes_gen << "\n";
			// Buscar la meta en path para reconstruir
			P<N> target;
			for (const auto& pair : path) {
				if (pair.first.isDone()) { target = pair.first; break; }
			}
			if(target.isDone()) reconstruct(path, target);
			std::cout << "FOUND in " << (end-start).count()/1000000.0 << "ms\n";
			return;
		}
		if (t >= 1e9 || nodes_gen > NODELIMIT) {
			std::cout << "NOT FOUND OR NODE LIMIT REACHED!!!!\n";
			return;
		}
		bound = t; // Incrementar la profundidad de búsqueda al menor límite excedido
	}
}

// --- Beam Search ---
template<int N>
void BeamSearch(const P<N>& root, int beam_width) {
	std::cout << "===========================\nRunning Beam Search (Width: " << beam_width << ")...\n";
	auto start = std::chrono::high_resolution_clock::now();

	std::vector<P<N>> current_level;
	std::unordered_set<P<N>> visited;
	std::unordered_map<P<N>, P<N>> path;
	std::size_t uniqueUsefulNodesGenerated = 1;

	current_level.push_back(root);
	visited.insert(root);

	while(!current_level.empty()) {
		std::vector<P<N>> next_level;
		for(const auto& node : current_level) {
			if(node.isDone()) {
				auto end = std::chrono::high_resolution_clock::now();
				reconstruct(path, node);
				std::cout << "Unique Nodes Visited: " << visited.size() << "\n";
				std::cout << "FOUND in " << (end-start).count()/1000000.0 << "ms\n";
				return;
			}

			std::vector<DIR> children;
			node.getCandidates(children);
			for(auto dir : children) {
				P<N> child(node);
				child.move(dir);
				if(visited.find(child) == visited.end()) {
					visited.insert(child);
					path[child] = node;
					next_level.push_back(child);
					uniqueUsefulNodesGenerated++;
				}
			}
		}

		if(uniqueUsefulNodesGenerated > NODELIMIT) {
			std::cerr << "max nodes reached, aborting\n";
			return;
		}

		// Ordenar nivel siguiente por h2 y truncar a beam_width
		std::sort(next_level.begin(), next_level.end(), [](const P<N>& a, const P<N>& b) {
			return a.H2() < b.H2();
		});

		if(next_level.size() > static_cast<std::size_t>(beam_width)) {
			next_level.resize(beam_width);
		}
		current_level = std::move(next_level);
	}
	std::cout << "NOT FOUND!!!!\n";
}

// --- RBFS Auxiliar ---
template<int N>
std::pair<P<N>, int> RBFS_Recursive(const P<N>& node, const P<N>* parent, int g, int f_node, int f_limit, 
                                    std::unordered_map<P<N>, P<N>>& path, std::size_t& nodes_gen) {
	if (node.isDone()) return {node, -1}; // -1 es la señal de éxito
	
	std::vector<DIR> candidates;
	node.getCandidates(candidates);
	
	std::vector<std::pair<P<N>, int>> successors;
	for(auto dir : candidates) {
		P<N> child(node);
		child.move(dir);
		
		// 1. PODA DEL PADRE: Evitar retroceso inmediato (oscilación)
		if (parent != nullptr && child == *parent) continue;
		
		nodes_gen++;
		// 2. MONOTONICIDAD: El 'f' del hijo es el máximo entre su propio coste y el heredado
		int child_f = std::max(g + 1 + child.H2(), f_node);
		successors.push_back({child, child_f});
	}

	if(successors.empty()) return {node, 1e9}; // Callejón sin salida

	while(true) {
		if(nodes_gen > NODELIMIT) return {node, 1e9};

		// Ordenar sucesores de menor a mayor 'f'
		std::sort(successors.begin(), successors.end(), 
		          [](const auto& a, const auto& b){ return a.second < b.second; });

		auto& best = successors[0];
		
		// Si el mejor hijo supera el límite actual, retrocedemos propagando su coste
		if(best.second > f_limit) return {node, best.second};

		// El f_limit para el mejor hijo es el mínimo entre el límite actual y el hermano que le sigue
		int alternative_f = (successors.size() > 1) ? successors[1].second : 1e9;
		
		auto result = RBFS_Recursive(best.first, &node, g + 1, best.second, std::min(f_limit, alternative_f), path, nodes_gen);
		
		if(result.second == -1) {
			// 3. CAMINO SEGURO: Solo guardamos la ruta si la rama fue exitosa
			path[best.first] = node;
			return result; 
		}
		
		// Actualizar el costo 'f' del nodo explorado para la siguiente iteración del while
		best.second = result.second;
	}
}

// --- RBFS (Recursive Best-First Search) ---
template<int N>
void RBFS(const P<N>& root) {
	std::cout << "===========================\nRunning RBFS (h2)...\n";
	auto start = std::chrono::high_resolution_clock::now();
	
	std::unordered_map<P<N>, P<N>> path;
	std::size_t nodes_gen = 1;
	
	int f_root = root.H2();
	
	// Iniciamos la recursión pasando nullptr como padre inicial
	auto result = RBFS_Recursive<N>(root, nullptr, 0, f_root, 1e9, path, nodes_gen);

	if (result.second == -1) {
		auto end = std::chrono::high_resolution_clock::now();
		reconstruct(path, result.first);
		std::cout << "Nodes Generated: " << nodes_gen << "\n";
		std::cout << "FOUND in " << (end-start).count()/1000000.0 << "ms\n";
	} else {
		std::cout << "NOT FOUND OR NODE LIMIT REACHED!!!!\n";
	}
}

// --- SMA* (Simplified Memory-bounded A*) ---
template<int N>
void SMAStar(const P<N>& root, std::size_t memory_limit) {
	// SMA* requiere gestión manual de grafos acíclicos dirigidos y poda de hojas olvidadas.
	// Por restricciones de complejidad en esta implementación directa sobre el estado del nodo, 
	// se incluye una variante arquitectónica simplificada apoyada en A* que respeta un límite estricto de frontera.
	std::cout << "===========================\nRunning SMA* (Mem Limit: " << memory_limit << ")...\n";
	auto start = std::chrono::high_resolution_clock::now();

	// Utilizaremos un contenedor vector con sort continuo para simular la expulsión del peor nodo (memory bounds)
	std::vector<AStarNode<N>> open_list;
	std::unordered_map<P<N>, int> g_score;
	std::unordered_map<P<N>, P<N>> path;
	std::size_t uniqueUsefulNodesGenerated = 1;

	open_list.push_back({root, 0});
	g_score[root] = 0;

	while(!open_list.empty()) {
		// Ordenamos descendentemente, para extraer por el final (back) como el mejor nodo (menor f)
		std::sort(open_list.begin(), open_list.end(), std::greater<AStarNode<N>>());
		
		AStarNode<N> current = open_list.back();
		open_list.pop_back();

		if(current.state.isDone()) {
			auto end = std::chrono::high_resolution_clock::now();
			reconstruct(path, current.state);
			std::cout << "FOUND in " << (end-start).count()/1000000.0 << "ms\n";
			return;
		}

		std::vector<DIR> children;
		current.state.getCandidates(children);
		for(auto dir : children) {
			P<N> child(current.state);
			child.move(dir);
			int tentative_g = current.g + 1;

			if(g_score.find(child) == g_score.end() || tentative_g < g_score[child]) {
				g_score[child] = tentative_g;
				path[child] = current.state;
				open_list.push_back({child, tentative_g});
				uniqueUsefulNodesGenerated++;
			}
		}

		// Si excedemos la memoria acotada, podamos los nodos con el mayor costo f()
		if(open_list.size() > memory_limit) {
			// Volvemos a ordenar porque hemos añadido elementos
			std::sort(open_list.begin(), open_list.end(), std::greater<AStarNode<N>>());
			// Eliminamos del frente del vector (los de mayor f, ya que el orden es de mayor a menor f)
			open_list.erase(open_list.begin(), open_list.begin() + (open_list.size() - memory_limit));
		}

		if(uniqueUsefulNodesGenerated > NODELIMIT) {
			std::cerr << "max nodes reached, aborting\n";
			return;
		}
	}
	std::cout << "NOT FOUND!!!!\n";
}