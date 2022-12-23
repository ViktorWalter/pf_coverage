#pragma once
#include <limits>

//Costante positiva piccola
template<typename T>
constexpr T EPSILON = std::numeric_limits<T>::epsilon();
//----------------------------------------------------------------------------------
//Almost predicates(confronti in senso lato)
//Valore minore o all'incirca uguale
template<typename T>
constexpr bool almostLower(T lhs, T rhs) noexcept
{
    return lhs <= rhs + EPSILON<T>;
}
//Valore maggiore o all'incirca uguale
template<typename T>
constexpr bool almostGreater(T lhs, T rhs) noexcept
{
    return lhs >= rhs - EPSILON<T>;
}
//Valori sostanzialmente uguali
template<typename T>
constexpr bool almostEqual(T lhs, T rhs) noexcept
{   //valgono contemporaneamente almostLower e almostGreater
    return almostLower(lhs, rhs) && almostGreater(lhs, rhs);
}
//Valore all'incirca uguale a zero
template<typename T>
constexpr bool almostZero(T x) noexcept
{
    return almostEqual(x, static_cast<T>(0));
}
//Valore x all'incirca compreso tra a e b
template<typename T>
constexpr bool almostBetween(T x, T a, T b) noexcept
{
    return almostGreater(x, a) && almostLower(x, b);
}
//----------------------------------------------------------------------------------
//Strictly predicates(confronti in senso stretto)
//Valore strettamente minore
template<typename T>
constexpr bool strictlyLower(T lhs, T rhs) noexcept
{
    return lhs < rhs - EPSILON<T>;
}
//Valore strettamente maggiore
template<typename T>
constexpr bool strictlyGreater(T lhs, T rhs) noexcept
{
    return lhs > rhs + EPSILON<T>;
}
//Valore x strettamente compreso tra a e b
template<typename T>
constexpr bool strictlyBetween(T x, T a, T b) noexcept
{
    return strictlyGreater(x, a) && strictlyLower(x, b);
}

template<typename T>
float getDeterminant(std::vector<std::vector<T>> Matrix){
        //this function is written in c++ to calculate the determinant of matrix
        // it's a recursive function that can handle matrix of any dimension
        float det = 0; // the determinant value will be stored here
        if (Matrix.size() == 1)
        {
            return Matrix[0][0]; // no calculation needed
        }
        else if (Matrix.size() == 2)
        {
            //in this case we calculate the determinant of a 2-dimensional matrix in a 
            //default procedure
            det = (Matrix[0][0] * Matrix[1][1] - Matrix[0][1] * Matrix[1][0]);
            return det;
        }
        else
        {
            //in this case we calculate the determinant of a squared matrix that have 
            // for example 3x3 order greater than 2
            for (int p = 0; p < Matrix[0].size(); p++)
            {
                //this loop iterate on each elements of the first row in the matrix.
                //at each element we cancel the row and column it exist in
                //and form a matrix from the rest of the elements in the matrix
                std::vector<std::vector<float>> TempMatrix; // to hold the shaped matrix;
                for (int i = 1; i < Matrix.size(); i++)
                {
                    // iteration will start from row one cancelling the first row values
                    std::vector<float> TempRow;
                    for (int j = 0; j < Matrix[i].size(); j++)
                    {
                        // iteration will pass all cells of the i row excluding the j 
                        //value that match p column
                        if (j != p)
                        {
                           TempRow.push_back(Matrix[i][j]);//add current cell to TempRow 
                        }
                    }
                    if (TempRow.size() > 0)
                        TempMatrix.push_back(TempRow);
                    //after adding each row of the new matrix to the vector tempx
                    //we add it to the vector temp which is the vector where the new 
                    //matrix will be formed
                }
                det = det + Matrix[0][p] * pow(-1, p) * getDeterminant(TempMatrix);
                //then we calculate the value of determinant by using a recursive way
                //where we re-call the function by passing to it the new formed matrix
                //we keep doing this until we get our determinant
            }
            return det;
        }
    }

template<typename T>
std::vector<std::vector<T>> getTranspose(const std::vector<std::vector<T>> matrix1) {

    //Transpose-matrix: height = width(matrix), width = height(matrix)
    std::vector<std::vector<T>> solution(matrix1[0].size(), std::vector<T> (matrix1.size()));

    //Filling solution-matrix
    for(size_t i = 0; i < matrix1.size(); i++) {
        for(size_t j = 0; j < matrix1[0].size(); j++) {
            solution[j][i] = matrix1[i][j];
        }
    }
    return solution;
}

template<typename T>
std::vector<std::vector<T>> getCofactor(const std::vector<std::vector<T>> vect) {
    if(vect.size() != vect[0].size()) {
        throw std::runtime_error("Matrix is not quadratic");
    } 

    std::vector<std::vector<T>> solution(vect.size(), std::vector<T> (vect.size()));
    std::vector<std::vector<T>> subVect(vect.size() - 1, std::vector<T> (vect.size() - 1));

    for(std::size_t i = 0; i < vect.size(); i++) {
        for(std::size_t j = 0; j < vect[0].size(); j++) {

            int p = 0;
            for(size_t x = 0; x < vect.size(); x++) {
                if(x == i) {
                    continue;
                }
                int q = 0;

                for(size_t y = 0; y < vect.size(); y++) {
                    if(y == j) {
                        continue;
                    }

                    subVect[p][q] = vect[x][y];
                    q++;
                }
                p++;
            }
            solution[i][j] = pow(-1, i + j) * getDeterminant(subVect);
        }
    }
    return solution;
}

template<typename T>
std::vector<std::vector<T>> getInverse(const std::vector<std::vector<T>> vect) {
    if(getDeterminant(vect) == 0) {
        throw std::runtime_error("Determinant is 0");
    } 

    float d = 1.0/getDeterminant(vect);
    std::vector<std::vector<T>> solution(vect.size(), std::vector<T> (vect.size()));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] = vect[i][j]; 
        }
    }

    solution = getTranspose(getCofactor(solution));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] *= d;
        }
    }

    return solution;
}

template<typename T>
std::vector<std::vector<T>> multiplyMatrices(std::vector<std::vector<T>> m1, std::vector<std::vector<T>> m2)
{
    std::vector<std::vector<T>> solution(m1.size(), std::vector<T> (m2[0].size()));
    for(size_t i = 0; i < m1.size(); i++) {
        for(size_t j = 0; j < m2[0].size(); j++) {
            for(size_t k = 0; k < m1[0].size(); k++) {
                solution[i][j] += m1[i][k] * m2[k][j];
            }
        }
    }
    return solution;
}