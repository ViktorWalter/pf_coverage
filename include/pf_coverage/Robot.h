#pragma once
// STL
#include <vector>
#include <iostream>
// My includes
#include "Vettore.h"
#include "Box.h"


//Classe Diagram --> DCEL Data Structure
template<typename T>
class Robot
{
public:
    struct Pose{
        Vector2<T> centre;      //Coordinate x,y del centro
        //friend Robot;
        //T orientation;        //Angolo orientamento
    };
    T z_max { 30.0 };           //valore di riferimento nel caso in cui abbiamo double

    //costruttore tipo 1
    Robot(T x_pose,T y_pose, T z_max = 30.0)
    {
        //Pose {{x_pose, y_pose}};
        mPose = Robot::Pose{{x_pose, y_pose}};
    };

    //costruttore tipo 2, con valori di default
    Robot(Vector2<T> centre = {0.0, 0.0}, T z_max = 30.0) : Pose {centre(centre)}
    {
        mPose = Robot::Pose{centre};
    }

    //distruttore
    ~Robot(){};

    Pose getPose(){
        return this->mPose;
    }

    //Funzione che rielabora il vettore input dei punti globali in coordinate locali per il singolo punto mainPoint
    std::vector<Vector2<T>> reworkPointsVector(const std::vector<Vector2<T>>& points, Vector2<T> mainPoint)
    {
        auto rwrk_points = std::vector<Vector2<T>>();

        //Trasformazione di coordinate (esterno al costruttore del diagramma)
        for(const auto& point : points){
            auto rwrk_point = Vector2<T>{point.x-mainPoint.x,point.y-mainPoint.y};
            rwrk_points.push_back(rwrk_point);
        }

        //stampa di debug per checkare se la funzione fa il suo compito
        std::cout << "\nCoordinate locali punti: " << std::endl;
        for(std::size_t i=0; i<rwrk_points.size(); ++i){
            std::cout << rwrk_points[i] << std::endl;
        }

        return rwrk_points;
    }

    //Funzione filtraggio/esclusione punti esterni alla RangeBox (simula azione sensore del robot)
    std::vector<Vector2<T>> filterPointsVector(const std::vector<Vector2<T>>& rwrk_points, Box<T> RangeBox)
    {
        auto flt_points = std::vector<Vector2<T>>();

        //Filtraggio
        for(std::size_t i=0; i<rwrk_points.size(); ++i){
            auto flt_point = rwrk_points[i];
            //Mantiene solo i punti vicini
            if((RangeBox.contains(rwrk_points[i])) && (rwrk_points[i]!=Vector2<T>{0.0,0.0})){
                flt_points.push_back(flt_point);
            }
        }

        //stampa di debug per checkare se la funzione fa il suo compito
        std::cout << "\nPunti vicini rilevati: " << std::endl;
        for(std::size_t i=0; i<flt_points.size(); ++i){
            std::cout << "Punto " << i << flt_points[i] << std::endl;
        }

        return flt_points;
    }

    Box<T> adjustRangeBox(Box<T> RangeBox, Vector2<T> point, const double ROBOT_RANGE, Box<T> AreaBox)
    {
        auto AdjRangeBox = RangeBox;    //default

        //Adattamento ai limiti del campo (AreaBox)
        if(point.x - AreaBox.left < ROBOT_RANGE){
            AdjRangeBox.left = AreaBox.left - point.x;
        }
        if(point.y - AreaBox.bottom < ROBOT_RANGE){
            AdjRangeBox.bottom = AreaBox.bottom - point.y;
        }
        if(AreaBox.right - point.x < ROBOT_RANGE){
            AdjRangeBox.right = AreaBox.right - point.x;
        }
        if(AreaBox.top - point.y < ROBOT_RANGE){
            AdjRangeBox.top = AreaBox.top - point.y;
        }

        return AdjRangeBox;
    }

    T adapt_gauss_deviation(T x,T y, Vector2<T> mean, T curr_var)
    {
        auto z1 = abs((x - mean.x) / curr_var);
        auto z2 = abs((y - mean.y) / curr_var);
        auto z = sqrt(pow(z1, 2) + pow(z2, 2));

        if (z <= this->z_max)
        {
            return curr_var;
        } else {
            return abs(sqrt(pow(x-mean.x, 2) + pow(y-mean.y, 2)) / this->z_max);
        }
    }

private:
    Pose mPose;
};