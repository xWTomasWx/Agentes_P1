#pragma once
#include <time.h>
#include <stdlib.h>
#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <chrono>

enum DIR{up,right,down,left};
extern int shuffleIterations;

template<int N> class P{
protected:
	std::array<int,N> brd;
	int x, y, sqrtN;
	int h;
	int h_manhattan; 

public:
	P();
	void move( DIR d );
	void getCandidates( std::vector<DIR>& v ) const;
	void drawBrd() const;
	void getTiles( std::vector<int>& p, std::vector<DIR>& v ) const;
	bool isDone() const;

	bool operator==(const P<N>& rhs) const{return brd==rhs.brd;}
	friend std::size_t std::hash<P<N>>::operator()( const P<N> & board) const noexcept;

	int H()const{return h;}
	int calcH()const;

	int H2()const{return h_manhattan;}
	int calcManhattan()const;
};
