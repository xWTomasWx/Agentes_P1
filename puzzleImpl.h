#pragma once
#include "puzzle.h"

/*template<int N>
P<N>::P(): x(std::sqrt(N)-1),y(std::sqrt(N)-1),sqrtN(std::sqrt(N)){
	int i = 1;
	std::vector<DIR> v;
	for( ; i < N; i++ ) {
		brd[i - 1] = i;
	}
	brd[N-1] = 0;
	for( i = 0; i < shuffleIterations; i++ ) {
		getCandidates( v );
		move( v[rand() % v.size()] );
		v.clear();
	}
	h=calcH();
}*/

template<int N>
int P<N>::calcH()const{
	int count = 0;
	for(int i=1 ; i < N; i++ ) {
		if(brd[i - 1] != i) count++;
	}
	return count;
}

/*template<int N>
void P<N>::move( DIR d ) {
	int t = x + y * sqrtN;
	switch( d ) {
		case up: y--; break;
		case right: x++; break;
		case down: y++; break;
		case left: x--;
	}
	brd[t] = brd[x + y * sqrtN];
	brd[x + y * sqrtN] = 0;
	h=calcH();
}*/

template<int N>
void P<N>::getCandidates( std::vector<DIR>& v ) const{
	if( x < sqrtN-1 ) v.push_back( right );
	if( x > 0 ) v.push_back( left );
	if( y < sqrtN-1 ) v.push_back( down );
	if( y > 0 ) v.push_back( up );
}

template<int N>
void P<N>::drawBrd() const{
	int r; std::cout << "\n\n";
	for( int y = 0; y < sqrtN; y++ ) {
		for( int x = 0; x < sqrtN; x++ )
			std::cout << "+----";
		std::cout<<"+\n";
		for( int x = 0; x < sqrtN; x++ ) {
			r = brd[x + y * sqrtN];
			std::cout << "| ";
			if( r < 10 ) std::cout << " ";
			if( !r ) std::cout << "  ";
			else std::cout << r << " ";
		}
		std::cout << "|\n";
	}
	for( int x = 0; x < sqrtN; x++ )
		std::cout << "+----";
	std::cout<<"+\n";
}

template<int N>
void P<N>::getTiles( std::vector<int>& p, std::vector<DIR>& v ) const{
	for( unsigned int t = 0; t < v.size(); t++ ) {
		int xx = x, yy = y;
		switch( v[t] ) {
			case up: yy--; break;
			case right: xx++; break;
			case down: yy++; break;
			case left: xx--;
		}
		p.push_back( brd[xx + yy * sqrtN] );
	}
}

template<int N>
bool P<N>::isDone() const{
	for( int i = 0; i < N-1; i++ ) {
		if( brd[i] != i + 1 ) return false;
	}
	return true;
}

namespace std
{
	template<int N> struct hash<P<N>>
	{
		std::size_t operator()( const P<N> & board) const noexcept
		{
			std::size_t hash=0;
			for(int i=0;i<N; i++){
				hash ^= (std::size_t)board.brd[i] << (i % (sizeof(std::size_t)*8));
			}
			return hash;
		}
	};
}


// --- Añadir al final del archivo ---
template<int N>
int P<N>::calcManhattan() const{
	int dist = 0;
	for(int i = 0; i < N; i++) {
		int val = brd[i];
		if(val != 0) { // El espacio en blanco (0) no se cuenta en la distancia Manhattan
			int targetX = (val - 1) % sqrtN;
			int targetY = (val - 1) / sqrtN;
			int currentX = i % sqrtN;
			int currentY = i / sqrtN;
			dist += std::abs(targetX - currentX) + std::abs(targetY - currentY);
		}
	}
	return dist;
}

// --- Modificar el constructor P() ---
template<int N>
P<N>::P(): x(std::sqrt(N)-1),y(std::sqrt(N)-1),sqrtN(std::sqrt(N)){
	int i = 1;
	std::vector<DIR> v;
	for( ; i < N; i++ ) {
		brd[i - 1] = i;
	}
	brd[N-1] = 0;
	for( i = 0; i < shuffleIterations; i++ ) {
		getCandidates( v );
		move( v[rand() % v.size()] );
		v.clear();
	}
	h = calcH();
	h_manhattan = calcManhattan(); // Inicializar
}

// --- Modificar P<N>::move ---
template<int N>
void P<N>::move( DIR d ) {
	int t = x + y * sqrtN;
	switch( d ) {
		case up: y--; break;
		case right: x++; break;
		case down: y++; break;
		case left: x--;
	}
	brd[t] = brd[x + y * sqrtN];
	brd[x + y * sqrtN] = 0;
	h = calcH();
	h_manhattan = calcManhattan(); // Actualizar en cada movimiento
}