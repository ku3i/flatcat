#ifndef VECTOR_H
#define VECTOR_H


class vector
{
    public:
        vector(unsigned int size);
        ~vector();
        unsigned int size(void);
        double& operator() (unsigned int index);
        double operator() (unsigned int index) const;
        void set(unsigned int index, double value);
        void multiply_by_scalar(double factor);
        void multiply_by_vector_elementwise(vector v);
        double length(void);
        vector norm(void);

    private:
        unsigned int N; // size of vector
        double *value;
};

class matrix
{
    public:
        matrix();
        matrix(unsigned int rows, unsigned int cols); // constructor
        double& operator() (unsigned int row, unsigned int col);
        double  operator() (unsigned int row, unsigned int col) const;
        matrix(matrix const& M);                // copy constructor
        matrix& operator= (matrix const& M);    // assignment operator

        ~matrix();
        double operator[](unsigned int index);

        unsigned int rows(void);
        unsigned int columns(void);
        void multiply_by_scalar(double factor);
        void multiply_by_matrix_elementwise(matrix M);
    private:
        unsigned int ROWS, COLS;
        double **value;
};


double inner_product(vector a, vector b);
matrix outer_product(vector a, vector b);

vector operator* (matrix M, vector a);

#endif // VECTOR_H
