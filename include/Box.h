#pragma once
// STL
#include <array>
// My includes
#include "Vettore.h"
#include "Utility.h"

//Prototipo classe (necessario perché richiamata nella classe Box)
template<typename T>
class Diagram;
//Prototipo classe (necessario perché richiamata nella classe Box)
template<typename T>
class FortuneAlgorithm;

//Classe Box --> Box Rettangolare
template<typename T>
class Box
{
public:
    //Variabili della bounding box quadrata/rettangolare
    T left;     //x-left
    T bottom;   //y-bottom
    T right;    //x-right
    T top;      //y-top

    //Metodo per verificare se un punto è contenuto nella box
    bool contains(const Vector2<T>& point) const
    {
        return almostBetween(point.x, left, right) && almostBetween(point.y, bottom, top);
    }

private:
    //Friendship con altre classi per dar loro accesso a questa sezione private
    friend Diagram<T>;
    friend FortuneAlgorithm<T>;

    //Variabile enum che identifica il lato della box
    enum class Side : int {Left, Bottom, Right, Top};
    //Struct intersezione
    struct Intersection
    {
        Side side;          //lato della box coinvolto nell'intersezione
        Vector2<T> point;   //Punto di intersezione
    };

    //Metodo per capire su quale lato della box avviene la prima intersezione di un vettore con la box
    Intersection getFirstIntersection(const Vector2<T>& origin, const Vector2<T>& direction) const
    {
        //origin must be in the box
        auto intersection = Intersection{};
        auto t = std::numeric_limits<T>::infinity();
        //Caso1: il vettore procede verso x positive
        if (direction.x > static_cast<T>(0.0))
        {
            t = (right - origin.x) / direction.x;   //parametro normalizzatore (di quanto esce il vettore)
            intersection.side = Side::Right;        //lato box coinvolto nell'intersezione
            intersection.point = origin + t * direction;    //punto di intersezione (vedi quaderno)
        }
        //Caso2 : il vettore procede verso x negative
        else if (direction.x < static_cast<T>(0.0))
        {
            t = (left - origin.x) / direction.x;    //parametro normalizzatore (di quanto esce il vettore)
            intersection.side = Side::Left;         //lato box coinvolto nell'intersezione
            intersection.point = origin + t * direction;    //punto di intersezione (vedi quaderno)
        }
        //Caso3: il vettore procede verso y positive
        if (direction.y > static_cast<T>(0.0))
        {   //Check se l'intersezione avviene con lato top anziché con left
            auto newT = (top - origin.y) / direction.y;
            if (newT < t)   //coordinata y finale ha peso maggiore della x finale
            {
                intersection.side = Side::Top;
                intersection.point = origin + newT * direction; //punto di intersezione (vedi quaderno)
            }
        }
        //Caso4: il vettore procede verso y negative
        else if (direction.y < static_cast<T>(0.0))
        {   //Check se l'intersezione avviene con lato bottom anziché con right
            auto newT = (bottom - origin.y) / direction.y;
            if (newT < t)   //coordinata y finale ha peso maggiore della x finale
            {
                intersection.side = Side::Bottom;
                intersection.point = origin + newT * direction; //punto di intersezione (vedi quaderno)
            }
        }
        return intersection;
    }

    //Metodo per trovare il numero delle intersezioni del diagramma con la box
    int getIntersections(const Vector2<T>& origin, const Vector2<T>& destination, std::array<Intersection, 2>& intersections) const
    {
        //Se l'intersezione è un corner della box, entrambe le intersezioni sono uguali
        auto direction = destination - origin;  //vettore direzione
        auto t = std::array<T, 2>();            //valori dell'array forniscono una misura della distanza dell'intersezione dall'origine degli assi
        auto i = std::size_t(0); //Indice dell'intersezione corrente
        //Intersezione con side left della box
        if (strictlyLower(origin.x, left) || strictlyLower(destination.x, left))
        {
            t[i] = (left - origin.x) / direction.x;
            if (strictlyBetween(t[i], static_cast<T>(0.0), static_cast<T>(1.0)))
            {
                intersections[i].side = Side::Left;
                intersections[i].point = origin + t[i] * direction;
                //Controlliamo che l'intersezione sia all'interno dei limiti y della box
                if (almostBetween(intersections[i].point.y, bottom, top))
                    ++i;    //incremento solo se intersezione è all'interno dei limiti y della box
            }
        }
        //Intersezione con side right della box
        if (strictlyGreater(origin.x, right) || strictlyGreater(destination.x, right))
        {
            t[i] = (right - origin.x) / direction.x;
            if (strictlyBetween(t[i], static_cast<T>(0.0), static_cast<T>(1.0)))
            {
                intersections[i].side = Side::Right;
                intersections[i].point = origin + t[i] * direction;
                //Controlliamo che l'intersezione sia all'interno dei limiti y della box
                if (almostBetween(intersections[i].point.y, bottom, top))
                    ++i;    //incremento solo se intersezione è all'interno dei limiti y della box
            }
        }
        //Intersezione con side bottom della box
        if (strictlyLower(origin.y, bottom) || strictlyLower(destination.y, bottom))
        {
            t[i] = (bottom - origin.y) / direction.y;
            if (i < 2 && strictlyBetween(t[i], static_cast<T>(0.0), static_cast<T>(1.0)))
            {
                intersections[i].side = Side::Bottom;
                intersections[i].point = origin + t[i] * direction;
                //Controlliamo che l'intersezione sia all'interno dei limiti x della box
                if (almostBetween(intersections[i].point.x, left, right))
                    ++i;    //incremento solo se intersezione è all'interno dei limiti x della box
            }
        }
        //Intersezione con side top della box
        if (strictlyGreater(origin.y, top) || strictlyGreater(destination.y, top))
        {
            t[i] = (top - origin.y) / direction.y;
            if (i < 2 && strictlyBetween(t[i], static_cast<T>(0.0), static_cast<T>(1.0)))
            {
                intersections[i].side = Side::Top;
                intersections[i].point = origin + t[i] * direction;
                //Controlliamo che l'intersezione sia all'interno dei limiti x della box
                if (almostBetween(intersections[i].point.x, left, right))
                    ++i;    //incremento solo se intersezione è all'interno dei limiti x della box
            }
        }
        //Ordiniamo le intersezioni dalla più vicina alla più lontana(swap)
        if (i == 2 && t[0] > t[1])
            std::swap(intersections[0], intersections[1]);
        return i;
    }
};
