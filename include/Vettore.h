#pragma once
// STL
#include <ostream>
#include <cmath>

//Dichiarazioni
template<typename T>
class Vector2;
template<typename T>
Vector2<T> operator-(Vector2<T> lhs, const Vector2<T>& rhs);

//Classe Vector2 --> per definire i punti(siti e vertici)
template<typename T>
class Vector2
{
public:
    //Variabili interne
    T x;    //coordinata x puntata dal vettore
    T y;    //coordinata y puntata dal vettore

    //Costruttore
    Vector2<T>(T x = 0.0, T y = 0.0) : x(x), y(y)
    {}


    //Overload operatori unari
    //Opposto di un vettore
    Vector2<T> operator-() const
    {
        return Vector2<T>(-x, -y);  //inversione di coordinate
    }
    //Somma del vettore per un altro vettore
    Vector2<T>& operator+=(const Vector2<T>& other)
    {
        x += other.x;   //somma coordinate x dei vettori
        y += other.y;   //somma coordinate y dei vettori
        return *this;   //restituisce il puntatore al vettore/punto risultante
    }
    //Differenza tra il vettore e un altro vettore
    Vector2<T>& operator-=(const Vector2<T>& other)
    {
        x -= other.x;   //differenza coordinate x dei vettori
        y -= other.y;   //differenza coordinate y dei vettori
        return *this;   //restituisce il puntatore al vettore/punto risultante
    }
    //Prodotto del vettore per uno scalare t
    Vector2<T>& operator*=(T t)
    {
        x *= t;         //prodotto coordinata x per lo scalare t
        y *= t;         //prodotto coordinata y per lo scalare t
        return *this;   //restituisce il puntatore al vettore/punto
    }

    //Metodi Getter
    //Vettore ortogonale rispetto al prodotto scalare
    Vector2<T> getOrthogonal() const
    {
        return Vector2<T>(-y, x); //restituisce il vettore ortogonale
    }
    //Calcolo della norma euclidea del vettore
    T getNorm() const
    {
        return std::sqrt(x * x + y * y);
    }
    //Calcolo distanza euclidea tra il vettore e un altro vettore (vettore da intendersi come punto in questo caso)
    T getDistance(const Vector2<T>& other) const
    {
        return (*this - other).getNorm();
    }
    //Calcolo del prodotto vettoriale con un altro vettore
    T getDet(const Vector2<T>& other) const
    {   //utile al calcolo del centroide
        return x * other.y - y * other.x;
    }
};

//Overload operatori binari
//Vettore somma di due vettori
template<typename T>
Vector2<T> operator+(Vector2<T> lhs, const Vector2<T>& rhs)
{
    lhs += rhs;
    return lhs;
}
//Vettore differenza di due vettori
template<typename T>
Vector2<T> operator-(Vector2<T> lhs, const Vector2<T>& rhs)
{
    lhs -= rhs;
    return lhs;
}
//Vettore prodotto di un vettore per uno scalare
template<typename T>
Vector2<T> operator*(T t, Vector2<T> vec)
{
    vec *= t;
    return vec;
}
//Vettore prodotto di uno scalare per un vettore
template<typename T>
Vector2<T> operator*(Vector2<T> vec, T t)
{
    return t * vec;
}

template<typename T>
bool operator==(const Vector2<T>& vec1, const Vector2<T>& vec2)
{
    return ((vec1.x == vec2.x) && (vec1.y == vec2.y));
}

template<typename T>
bool operator!=(const Vector2<T>& vec1, const Vector2<T>& vec2)
{
    return ((vec1.x != vec2.x) || (vec1.y != vec2.y));
}

//Overload operatore di stream/stampa per un oggetto Vector2
template<typename T>
std::ostream& operator<<(std::ostream& os, const Vector2<T>& vec)
{
    os << "(" << vec.x << ", " << vec.y << ")"; //Stampa delle coordinate del punto individuato dal vettore
    return os;
}
