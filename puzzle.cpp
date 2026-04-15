#include "puzzleImpl.h"
#include "solverImpl.h"

std::size_t NODELIMIT = 10000000;
int shuffleIterations = 25;

template<int N>
void runSolvers(int iters){
	shuffleIterations = iters;
	srand((unsigned)time(0));
	P<N> p;
	p.drawBrd();
	std::cout << "=== Size=" << N << "  shuffleIterations=" << iters << " ===" << std::endl;
	greedy(p);
	BFS(p);
	AStar(p);
	IDAStar(p);
	BeamSearch(p, 100); // Puedes ajustar el ancho del haz según sea necesario
	RBFS(p);
	SMAStar(p, 100000); // Puedes ajustar el límite de memoria según sea necesario

}

int main() {
	runSolvers<16>(25);
	return 0;
}
