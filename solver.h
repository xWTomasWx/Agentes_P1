#pragma once
#include "puzzle.h"
#include <cstddef>
#include <unordered_map>

extern std::size_t NODELIMIT;

template<int N> int  reconstruct(const std::unordered_map<P<N>,P<N>> &path, const P<N> &node);
template<int N> void BFS(const P<N>& root);
template<int N> void greedy(const P<N>& root);
template<int N> void AStar(const P<N>& root);
template<int N> void IDAStar(const P<N>& root);
template<int N> void BeamSearch(const P<N>& root, int beam_width = 100);
template<int N> void RBFS(const P<N>& root);
template<int N> void SMAStar(const P<N>& root, std::size_t memory_limit = 100000);