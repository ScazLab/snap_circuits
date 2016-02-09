#include "snapCircuits/snapCircuitsBoard.h"

using namespace std;
using namespace snapCircuits;

snapCircuitsBoard::snapCircuitsBoard()
{
    set_n_rows_and_cols(N_ROWS,N_COLS);
}

snapCircuitsBoard::snapCircuitsBoard(int _n_rows, int _n_cols)
{
    set_n_rows_and_cols(_n_rows,_n_cols);
}

bool snapCircuitsBoard::set_n_rows_and_cols(const int &_r, const int &_c)
{
    return set_n_rows(_r) && set_n_cols(_c);
}
