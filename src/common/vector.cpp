#include "vector.h"
#include "basic.h"

/**
TODO: this file is kind of obsolete and untested, DO NOT USE */

/* methods for class 'vector' */

vector::vector(unsigned int vsize)
{
    N = vsize;
    value = init_double(N);
}

vector::~vector()
{
    free(value);
    value = NULL;
}

unsigned int
vector::size()
{
    return N;
}

inline double&
vector::operator() (unsigned int index)
{
    return value[index];
}

inline double
vector::operator() (unsigned int index) const
{
    return value[index];
}

void
vector::set(unsigned int index, double new_value)
{
    if (index < N)
        value[index] = new_value;
}

double
vector::length(void)
{
    double len = .0;
    for (unsigned int i = 0; i < N; i++)
        len += value[i] *  value[i];
    return sqrt(len);
}

void
vector::multiply_by_scalar(double factor)
{
    for (unsigned int i = 0; i < N; i++)
        value[i] *= factor;
}

void
vector::multiply_by_vector_elementwise(vector v)
{
    if (v.size() != N) err_msg(__FILE__, __LINE__, "Vectors do not fit for vector multiplication.");

    for (unsigned int i = 0; i < N; i++)
        value[i] *= v(i);
}

/* methods for class 'matrix' */

matrix::matrix() {};

matrix::matrix(unsigned int rows, unsigned int cols)
{
    if (0 == rows) err_msg(__FILE__, __LINE__, "Number of rows is zero.");
    if (0 == cols) err_msg(__FILE__, __LINE__, "Number of columns is zero.");
    ROWS = rows;
    COLS = cols;
    value = init_double(ROWS, COLS);
}

matrix:: ~matrix()
{
    free(value);
    value = NULL;
}

inline double&
matrix::operator() (unsigned int row, unsigned int col)
{
    if (row >= ROWS) err_msg(__FILE__, __LINE__, "Row index out of bounds.");
    if (col >= COLS) err_msg(__FILE__, __LINE__, "Column index out of bounds.");
    return value[row][col];
}

inline double
matrix::operator() (unsigned row, unsigned col) const
{
    if (row >= ROWS) err_msg(__FILE__, __LINE__, "Row index out of bounds.");
    if (col >= COLS) err_msg(__FILE__, __LINE__, "Column index out of bounds.");
    return value[row][col];
}

unsigned int
matrix::columns(void)
{
    return COLS;
}

unsigned int
matrix::rows(void)
{
    return ROWS;
}

void
matrix::multiply_by_scalar(double factor)
{
    for (unsigned int i = 0; i < ROWS; i++)
        for (unsigned int j = 0; j < COLS; j++)
            value[i][j] *= factor;
}

void
matrix::multiply_by_matrix_elementwise(matrix M)
{
    if (M.columns() != COLS) err_msg(__FILE__, __LINE__, "Matrices do not match in the number of columns.");
    if (M.rows() != ROWS) err_msg(__FILE__, __LINE__, "Matrices do not match in the number of rows.");

    for (unsigned int i = 0; i < ROWS; i++)
        for (unsigned int j = 0; j < COLS; j++)
            value[i][j] *= M(i,j);
}

/* operators for two vectors */

double
inner_product(vector a, vector b)
{
    double ip = .0;
    unsigned int N = a.size();
    if (N != b.size()) err_msg(__FILE__, __LINE__, "Vectors do not match in size.");
    for (unsigned int i = 0; i < N; i++)
        ip += a(i) * b(i);

    return ip;
}


matrix
outer_product(vector a, vector b)
{
    unsigned int N = a.size();
    if (N != b.size()) err_msg(__FILE__, __LINE__, "Vectors do not match in size.");

    matrix M(N,N);

    for (unsigned int i = 0; i < N; i++)
        for (unsigned int j = 0; j < N; j++)
            M(i,j) = a(i) * b(j);
    return M;
}

vector
operator* (matrix A, vector x)
{
    unsigned int N = A.rows();
    unsigned int M = A.columns();
    if (M != x.size()) err_msg(__FILE__, __LINE__, "The number of columns of the matrix and the size of the vector do not match.");
    vector y(N);

    for (unsigned int i = 0; i < N; i++) //rows
        for (unsigned int j = 0; j < M; j++) //columns
            y(i) += A(i,j) * x(j);

    return y;
}


