#ifndef VORONOI_H_INCLUDED
#define VORONOI_H_INCLUDED

#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

// My includes
#include "FortuneAlgorithm.h"
#include "Utility.h"
#include <iostream>
// #include "/home/mattia/arscontrol_turtlebot/install/turtlebot3_msgs/include/turtlebot3_msgs/msg/gmm.hpp"
// #include "/home/mattia/arscontrol_turtlebot/install/turtlebot3_msgs/include/turtlebot3_msgs/msg/gaussian.hpp"
#include "turtlebot3_msgs/msg/gmm.hpp"
#include "turtlebot3_msgs/msg/gaussian.hpp"


#define M_PI   3.14159265358979323846  /*pi*/

// const int DEBUG = 0;

float gauss3d_pdf(std::vector<float> mean, std::vector<std::vector<float>> var, std::vector<float> pt);
float multiple_gauss3d_pdf(std::vector<std::vector<float>> means_pt, std::vector<std::vector<std::vector<float>>> vars, std::vector<float> pt, std::vector<float> weights);
float gauss3d_pdf2(turtlebot3_msgs::msg::Gaussian gaussian, std::vector<float> pt);
float multiple_gauss3d_pdf2(turtlebot3_msgs::msg::GMM gmm, std::vector<float> pt);
float single_component_pdf(turtlebot3_msgs::msg::Gaussian gaussian, std::vector<float> pt);
float mixture_pdf(turtlebot3_msgs::msg::GMM gmm, std::vector<float> pt);
std::vector<float> computeGMMPolygonCentroid2(const Diagram<double> &polygon, turtlebot3_msgs::msg::GMM gmm, std::vector<Box<double>> ObstacleBoxes, float discretize_precision);

//***********************************************************************************************************************************
//-------------------------------- FUNZIONI AUSILIARIE INIZIALI (trasformazione coordinate + sensing) ---------------------------------
//Funzione che esclude dal calcolo i punti esterni ad AreaBox
template<typename T>
std::vector<Vector2<T>> adjustPointsVector(const std::vector<Vector2<T>>& points, Box<T> AreaBox)
{
    auto adj_points = std::vector<Vector2<T>>();

    for(const auto& point : points){
        //Esclusione punti esterni
        if(AreaBox.contains(point)){
            adj_points.push_back(point);
        }
    }

    return adj_points;
}

//Funzione che rielabora il vettore input dei punti globali in coordinate locali per il singolo punto mainPoint
template<typename T>
std::vector<Vector2<T>> reworkPointsVector(const std::vector<Vector2<T>>& points, Vector2<T> mainPoint)
{
    auto rwrk_points = std::vector<Vector2<T>>();

    //Trasformazione di coordinate (esterno al costruttore del diagramma)
    for(const auto& point : points){
        auto rwrk_point = Vector2<T>{point.x-mainPoint.x,point.y-mainPoint.y};
        rwrk_points.push_back(rwrk_point);
    }

    //stampa di debug per checkare se la funzione fa il suo compito
    if (DEBUG >= 1)
    {
        std::cout << "\nCoordinate locali punti: " << std::endl;
        for(std::size_t i=0; i<rwrk_points.size(); ++i){
            std::cout << rwrk_points[i] << std::endl;
        }
    }

    return rwrk_points;
}

// Funzione per descrivere gli ostacoli in coordinate locali rispetto a posizione robot
template<typename T>
Box<T> reworkObstacle(Box<T> ObstacleBox, Vector2<T> global_pos)
{
    Box<T> adjObstacleBox;

    // Get vertex coordinates
    Vector2<T> bl = {ObstacleBox.left, ObstacleBox.bottom};         // bottom-left point
    Vector2<T> tr = {ObstacleBox.right, ObstacleBox.top};         // top-right point

    // Define vector of vertexes
    std::vector<Vector2<T>> Vert = {bl,tr};

    // Rework points
    std::vector<Vector2<T>> adjVert = reworkPointsVector(Vert, global_pos);

    adjObstacleBox.left = adjVert[0].x;
    adjObstacleBox.bottom = adjVert[0].y;
    adjObstacleBox.right = adjVert[1].x;
    adjObstacleBox.top = adjVert[1].y;

    // if (global_pos.y < adjObstacleBox.bottom)
    // {   
    //     adjObstacleBox.top = adjObstacleBox.top + 5.0;
    // }
    // if (global_pos.x < adjObstacleBox.left)
    // {   
    //     adjObstacleBox.right = adjObstacleBox.right + 5.0;
    // }
    // if (global_pos.y > adjObstacleBox.top)
    // {   
    //     adjObstacleBox.bottom = adjObstacleBox.bottom - 5.0;
    // }
    // if (global_pos.x > adjObstacleBox.right)
    // {   
    //     adjObstacleBox.left = adjObstacleBox.left - 5.0;
    // }

    return adjObstacleBox;
}

//Funzione filtraggio/esclusione punti esterni alla RangeBox (simula azione sensore del robot)
template<typename T>
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
    if (DEBUG >= 1)
    {
        std::cout << "\nPunti vicini rilevati: " << std::endl;
        for(std::size_t i=0; i<flt_points.size(); ++i){
            std::cout << "Punto " << i << flt_points[i] << std::endl;
        }
    }

    return flt_points;
}

//Funzione filtraggio/esclusione punti esterni alla RangeBox (simula azione sensore del robot) 
//Return also the ids fo the neighbors
template<typename T>
std::pair<std::vector<Vector2<T>>, std::vector<int>> filterPointsVectorNeighbor(const std::vector<Vector2<T>>& rwrk_points, Box<T> RangeBox)
{
    auto flt_points = std::vector<Vector2<T>>();
    std::vector<int> neighbors = std::vector<int>();

    //Filtraggio
    for(std::size_t i=0; i<rwrk_points.size(); ++i){
        auto flt_point = rwrk_points[i];
        //Mantiene solo i punti vicini
        if((RangeBox.contains(rwrk_points[i])) && (rwrk_points[i]!=Vector2<T>{0.0,0.0})){
            flt_points.push_back(flt_point);
            neighbors.push_back(i);
        }
    }

    //stampa di debug per checkare se la funzione fa il suo compito
    if (DEBUG >= 1)
    {
        std::cout << "\nPunti vicini rilevati: " << std::endl;
        for(std::size_t i=0; i<flt_points.size(); ++i){
            std::cout << "Punto " << i << flt_points[i] << std::endl;
        }
    }

    return std::make_pair(flt_points, neighbors);
}

template<typename T>
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
//***********************************************************************************************************************************

//***********************************************************************************************************************************
//--------------------------------------------------- Diagramma Centralizzato ------------------------------------------------------
template<typename T>
Diagram<T> generateCentralizedDiagram(const std::vector<Vector2<T>>& points, Box<T> AreaBox)
{
    // Construct Diagram
    FortuneAlgorithm<T> algorithm(points, AreaBox);
    algorithm.construct();

    // Bound the diagram (Take the bounding box slightly bigger than the intersection box)
    algorithm.bound(Box<T>{-0.05 + AreaBox.left, -0.05 + AreaBox.bottom, 0.05 + AreaBox.right, 0.05 + AreaBox.top});
    Diagram<T> diagram = algorithm.getDiagram();

    // Intersect the diagram with a box
    if(diagram.getNbSites()!=1){
        diagram.intersect(AreaBox);
    }
    else{
        diagram.intersect_null(AreaBox);
    }

    return diagram;
}

//--------------------------------------------------- DIAGRAMMA DECENTRALIZZATO ----------------------------------------------------------------------------------------------------------------------
//Alla seguente funzione arrivano in input i punti gi� in coordinate locali e filtrati (quindi nel main dovr� essere preceduta dalle funzioni adjustRangeBox e reworkPointsVector(points, 0) mettendo come elemento 0 il punto di cui si vuole realizzare il diagramma decentralizzato
template<typename T>    //flt_points: punti gi� filtrati, RangeBox: per comodit�, point_global: informazione su posizione iniziale globale, ROBOT_RANGE e AREA_SIZE: informazioni utili per singolo robot
Diagram<T> generateDecentralizedDiagram(const std::vector<Vector2<T>>& flt_points, Box<T> RangeBox, const Vector2<T>& point_global, const double ROBOT_RANGE, Box<T> AreaBox)    //point_global corrisponde alle coordinate globali del punto centrale del diagramma rispetto all'area di lavoro (informazione conosciuta dal robot?)
{
    auto half_ROBOT_RANGE = ROBOT_RANGE/2;
    Box<double> HalfRangeBox{-half_ROBOT_RANGE, -half_ROBOT_RANGE, half_ROBOT_RANGE, half_ROBOT_RANGE};
    // Construct Diagram
    FortuneAlgorithm<T> algorithm(flt_points, HalfRangeBox, point_global);
    algorithm.construct();

    // Bound the diagram
    algorithm.bound(Box<T>{-half_ROBOT_RANGE-0.05, -half_ROBOT_RANGE-0.05, half_ROBOT_RANGE+0.05, half_ROBOT_RANGE+0.05});
    Diagram<T> diagram = algorithm.getDiagram();

    //adjustRangeBox andrebbe qui secondo me--> perch� il robot vede in base a RangeBox e aggiusta in AdjRangeBox in base alla conoscenza dei limiti del campo di lavoro e della sua posizione globale
    auto AdjRangeBox = adjustRangeBox(HalfRangeBox, point_global, half_ROBOT_RANGE, AreaBox);

    // Intersect the diagram with a box (diversificato in base al fatto che il diagramma abbia o meno altri siti oltre a quello centrale --> soluzione anti-crash)
    if(diagram.getNbSites()!=1){        //funzione di intersezione classica/originale nel caso di un numero di siti maggiore di 1
        diagram.intersect(AdjRangeBox);
    }
    else{                               //funzione anti-crash nel caso di diagramma con 1 solo sito, il diagramma coincide con la box esterna
        diagram.intersect_null(AdjRangeBox);    //OPPURE solo in questo caso RangeBox (usare Test Centroidi Decentralizzati nel main per far capire)
    }
    //Debug
    if (DEBUG >= 1)
    {
        std::cout << "Numero di siti: " << diagram.getNbSites() << std::endl;
        std::cout << "Numero di facce: " << diagram.getFaces().size() << std::endl;
        std::cout << "Numero di halfedge: " << diagram.getHalfEdges().size() << std::endl;
        std::cout << "Numero di vertici: " << diagram.getVertices().size() << std::endl;
    }

    // std::cout << "====== Vertici RangeBox: ========\n";
    // std::cout << AdjRangeBox.left << ", " << AdjRangeBox.bottom << std::endl;
    // std::cout << AdjRangeBox.right << ", " << AdjRangeBox.bottom << std::endl;
    // std::cout << AdjRangeBox.right << ", " << AdjRangeBox.top << std::endl;
    // std::cout << AdjRangeBox.left << ", " << AdjRangeBox.top << std::endl;

    return diagram;
}

template<typename T>
std::vector<Diagram<T>> generateDecentralizedDiagrams(const std::vector<Vector2<T>>& points, Box<T> RangeBox, const double ROBOT_RANGE, Box<T> AreaBox)
{
    //Vettore dei diagrammi di Voronoi
    auto diagrams = std::vector<Diagram<T>>();

    //Generatore iterativo sull'index
    for(std::size_t i=0; i<points.size(); ++i){
        if (DEBUG >= 1)
        {
            std::cout << "-------------------------------------\n";
            std::cout << "Diagramma " << i << " - " << "di " << points[i] << std::endl;
        }

        //Aggiustamento RangeBox in base ai limiti del campo --> fuori non dovrebbe rilevare punti
        //auto AdjRangeBox = adjustRangeBox(RangeBox, points[i], ROBOT_RANGE, AREA_SIZE);

        //Rielaborazione vettore "points" globale in coordinate locali
        auto rwrk_points = reworkPointsVector(points, points[i]);

        //Filtraggio siti esterni alla box (simula azione del sensore)
        auto flt_points = filterPointsVector(rwrk_points, /*Adj*/RangeBox);

        //Generazione Diagramma Decentralizzato
        auto diagram = generateDecentralizedDiagram(flt_points, /*Adj*/RangeBox, points[i], ROBOT_RANGE, AreaBox);
        diagrams.push_back(std::move(diagram));
    }

    return diagrams;
}
//***********************************************************************************************************************************

//***********************************************************************************************************************************
//----------------------------- Compute centroids - Uniform Distribution - Green's Theorem - Gauss Theorem --------------------------

//Funzione calcolo vettore dei centroidi dove ogni elemento del vettore � il centroide del site0 di ogni diagramma del vettore dei diagrammi
//Centroidi geometrici (densit� uniforme (fi=1)) --> una volta debuggato nel complesso spostare in Diagram.h per lavorare sul singolo sito
template<typename T>
std::vector<Vector2<T>> computeCentroids(const std::vector<Diagram<T>>& diagrams)
{
    auto centroids = std::vector<Vector2<T>>();
    
    for(const auto& diagram : diagrams){
        centroids.push_back(computeCentroid(diagram));
    }
    return centroids;
}

//Calcolo Centroide Geometrico del poligono definito dal diagramma voronoi - Green's theorem per il calcolo del centroide -
template<typename T>
Vector2<T> computeCentroid(const Diagram<T>& diagram)
{
    //Funzione calcolo centroide del mainSite (site 0) o equivalentemente della mainFace (face 0 associata al site 0)
    auto area = static_cast<T>(0.0);                        //inizializzazione area (double)
    auto centroid = Vector2<T>();                           //inizializzazione variabile centroide
    auto halfEdge = diagram.getFace(0)->outerComponent;     //prendiamo l'half-edge puntato dalla faccia
    //Compute centroid of the face
    do
    {
        auto det = halfEdge->origin->point.getDet(halfEdge->destination->point);    //prodotto vettoriale vettore origine e vettore destinazione dell'half-edge considerato
        area += det;                                                                //contributo al calcolo dell'area del poligono (singolo termine della sommatoria)
        centroid += (halfEdge->origin->point + halfEdge->destination->point) * det; //contributo al calcolo del centroide (singolo termine della sommatoria)
        halfEdge = halfEdge->next;                                                  //passaggio all'half-edge successivo della face 0
    } while(halfEdge != diagram.getFace(0)->outerComponent);    //ciclo do-while continua fino ad esaurimento degli half-edge della faccia considerato
    area *= 0.5;                                                //area del poligono/cella centrale del diagramma
    centroid *= 1.0 / (6.0 * area);                             //centroide del poligono/cella centrale del diagramma

    return centroid;    //centroide = spostamento locale rispetto a posizione corrente {0.0,0.0}
}
//***********************************************************************************************************************************

//***********************************************************************************************************************************
//----------------------------- Compute centroids - Gauss Distribution - Line Integration Method ------------------------------------
//Funzioni utili al calcolo dei centroidi nel caso di densit� non uniforme (Gaussian Density Function by Line Integration, vedi paper omonimo)

//Funzione per calcolo integrale definito
//Metodo Simpson (++ precisione) --> approssima in sottintervalli ciascuno dei quali � definito da una parabola (curva del secondo grado)
template<typename Function>
double computeIntegral(double a, double b, std::size_t numBins, Function f)
{
    double step = (b-a)/numBins;    //numBins: numero di sottointervalli, step: ampiezza sottointervallo
    double s = a;                   //a: primo estremo di integrazione
    double integral = 0;
    while(s < b){                   //b: secondo estremo di integrazione
        integral += step*(f(s) + f(s+step) + 4*f(0.5*(s+s+step)))/6.;  //integrale = somma delle aree dei singoli blocchetti/intervalli
        s += step;  //passaggio ad intervallo successivo
    }
    return integral;
}

//Funzione f(s) integranda della massa della cella
double integranda_mass(double s, Vector2<double> currentVertex, Vector2<double> nextVertex, Vector2<double> local_p_t, double var)
{
    double erf_term = std::erf(((nextVertex.x - currentVertex.x)*s + currentVertex.x - local_p_t.x)/((std::sqrt(2))*var));
    double pow_term = std::pow(((nextVertex.y - currentVertex.y)*s + currentVertex.y - local_p_t.y)/var, 2);
    double exp_term = std::exp(-0.5*pow_term);

    return erf_term*exp_term;
}
//Funzione f(s) integranda della coordinata x del centroide
double integranda_centroid_x(double s, Vector2<double> currentVertex, Vector2<double> nextVertex, Vector2<double> local_p_t, double var)
{
    double erf_term = std::erf(((nextVertex.y - currentVertex.y)*s + currentVertex.y - local_p_t.y)/((std::sqrt(2))*var));
    double pow_term = std::pow(((nextVertex.x - currentVertex.x)*s + currentVertex.x - local_p_t.x)/var , 2);
    double exp_term = std::exp(-0.5*pow_term);
    double lin_term = ((nextVertex.x - currentVertex.x)*s + currentVertex.x);

    return lin_term*exp_term*erf_term;
}
//Funzione f(s) integranda della coordinata x del centroide
double integranda_centroid_y(double s, Vector2<double> currentVertex, Vector2<double> nextVertex, Vector2<double> local_p_t, double var)
{
    double erf_term = std::erf(((nextVertex.x - currentVertex.x)*s + currentVertex.x - local_p_t.x)/((std::sqrt(2))*var));
    double pow_term = std::pow(((nextVertex.y - currentVertex.y)*s + currentVertex.y - local_p_t.y)/var , 2);
    double exp_term = std::exp(-0.5*pow_term);
    double lin_term = ((nextVertex.y - currentVertex.y)*s + currentVertex.y);

    return lin_term*exp_term*erf_term;
}

//Vettore dei centroidi non geometrici (densit� Gaussiana) in funzione dei vertici della cella e dei parametri della distribuzione (var, p_t)
template<typename T>
std::vector<Vector2<T>> computeGaussianCentroids(const std::vector<Diagram<T>>& diagrams, Vector2<T> p_t, double var)
{
    auto centroids = std::vector<Vector2<T>>();

    for(const auto& diagram : diagrams){
        centroids.push_back(computeGaussianCentroid(diagram, p_t, var));
    }

    return centroids;
}

template<typename T>
Vector2<T> computeGaussianCentroid(const Diagram<T>& diagram, Vector2<T> p_t, double var)
{
    //Calcolo p_t in coordinate locali (local_p_t);
    auto local_p_t = p_t - diagram.getGlobalPoint();
    //debug
    if (DEBUG >= 1)
    {
        std::cout << "Punto di interesse -->" << local_p_t << std::endl;
    }
    //Inizializzazioni massa e centroide
    auto mass = static_cast<T>(0.0);                        //inizializzazione double massa
    auto centroid = Vector2<T>();                           //inizializzazione centroide
    auto halfEdge = diagram.getFace(0)->outerComponent;     //prendiamo l'half-edge puntato dalla faccia
    do
    {   //MVi
        double integral = computeIntegral(0, 1, 500, [&](double s){return integranda_mass(s, halfEdge->origin->point, halfEdge->destination->point, local_p_t, var);});
        mass += integral*(halfEdge->destination->point.y - halfEdge->origin->point.y);
        //Cx
        double integral_x = computeIntegral(0, 1, 500, [&](double s){return integranda_centroid_x(s, halfEdge->origin->point, halfEdge->destination->point, local_p_t, var);});
        centroid.x += integral_x*(halfEdge->origin->point.x - halfEdge->destination->point.x);
        //Cy
        double integral_y = computeIntegral(0, 1, 500, [&](double s){return integranda_centroid_y(s, halfEdge->origin->point, halfEdge->destination->point, local_p_t, var);});
        centroid.y += integral_y*(halfEdge->destination->point.y - halfEdge->origin->point.y);

        //Passaggio all'half-edge successivo della face 0
        halfEdge = halfEdge->next;
    } while(halfEdge != diagram.getFace(0)->outerComponent);
    //Moltiplicazione per le costanti comuni
    mass *= (((std::sqrt(2*M_PI))*var)/2);
    centroid.x *= (1/mass)*(((std::sqrt(2*M_PI))*var)/2);
    centroid.y *= (1/mass)*(((std::sqrt(2*M_PI))*var)/2);

    return centroid;
}

//Vettore dei centroidi non geometrici due punti di interesse (densit� Gaussiana) in funzione dei vertici della cella e dei parametri della distribuzione (var, p_t)
template<typename T>
std::vector<Vector2<T>> compute2GaussianCentroids(const std::vector<Diagram<T>>& diagrams, const std::vector<Vector2<T>>& p_ts, const std::vector<double>& vars)
{
    //Vettore dei centroidi gaussiani
    auto centroids = std::vector<Vector2<T>>();

    //Per ogni diagramma del vettore dei diagrammi � necessario calcolare il centroide del sito centrale main (site 0)
    for(const auto& diagram : diagrams){
        centroids.push_back(compute2GaussianCentroid(diagram, p_ts, vars));  //inserimento nel vettore dei centroidi
    } //dopodich� si prosegue con il diagramma successivo

    return centroids;
}

template<typename T>
Vector2<T> compute2GaussianCentroid(const Diagram<T>& diagram, const std::vector<Vector2<T>>& p_ts, const std::vector<double>& vars)
{
    //Vettore dei punti interesse in coordinate locali
    auto local_p_ts = std::vector<Vector2<T>>();
    //Calcolo punti p_t in coordinate locali (local_p_t);
    for(const auto& p_t : p_ts){
        auto local_p_t = p_t - diagram.getGlobalPoint();        //trasformazione
        local_p_ts.push_back(local_p_t);                        //salvataggio nel vettore
        //debug
        if (DEBUG >= 1)
        {
            std::cout << "Punto di interesse -->" << local_p_t << std::endl;
        }
    }

    //Inizializzazioni massa e centroide
    auto mass = static_cast<T>(0.0);                        //inizializzazione double massa
    auto centroid = Vector2<T>();                           //inizializzazione centroide
    auto halfEdge = diagram.getFace(0)->outerComponent;     //prendiamo l'half-edge puntato dalla faccia
    for(std::size_t i=0; i<local_p_ts.size(); ++i){
        do
        {   //MVi
            double integral = computeIntegral(0, 1, 500, [&](double s){return integranda_mass(s, halfEdge->origin->point, halfEdge->destination->point, local_p_ts[i], vars[i]);});
            mass += integral*(halfEdge->destination->point.y - halfEdge->origin->point.y);
            //Cx
            double integral_x = computeIntegral(0, 1, 500, [&](double s){return integranda_centroid_x(s, halfEdge->origin->point, halfEdge->destination->point, local_p_ts[i], vars[i]);});
            centroid.x += integral_x*(halfEdge->origin->point.x - halfEdge->destination->point.x);
            //Cy
            double integral_y = computeIntegral(0, 1, 500, [&](double s){return integranda_centroid_y(s, halfEdge->origin->point, halfEdge->destination->point, local_p_ts[i], vars[i]);});
            centroid.y += integral_y*(halfEdge->destination->point.y - halfEdge->origin->point.y);

            //Passaggio all'half-edge successivo della face 0
            halfEdge = halfEdge->next;
        } while(halfEdge != diagram.getFace(0)->outerComponent);
        //Moltiplicazione per le costanti comuni
        mass *= (((std::sqrt(2*M_PI))*vars[i])/2);
        centroid.x *= (((std::sqrt(2*M_PI))*vars[i])/2);
        centroid.y *= (((std::sqrt(2*M_PI))*vars[i])/2);
    }
    centroid.x *= (1/mass);
    centroid.y *= (1/mass);

    return centroid;
}
//***********************************************************************************************************************************

//***********************************************************************************************************************************
//------------------------------- Calcolo centroide con Distribuzione Gaussiana discretizzando l'area ----------------------------------------------------------------------------------

//Jordan curve theorem:
//per verificare se un punto si trova dentro o fuori un poligono
//traccio una riga orizzontale (aumentando x e tenendo fissa y) che esce dal punto in questione
//se la riga interseca il poligono un numero pari di volte allora si trova FUORI dal poligono
//se la riga interseca il poligono un numero dispari di volte allora si trova DENTRO il poligono
//la variabile bool in viene cambiata ogni volta che vi � una intersezione con il poligono
template<typename T>
bool inPolygon(const Diagram<T> &polygon, Vector2<T> point)
{
    bool in = false;
    auto seed = polygon.getSite(0);
    auto halfedge = seed->face->outerComponent;

    //partendo da un lato ripercorro tutto il perimentro del poligono, vertice per vertice verificando la presenza di intersezioni
    do{
        if (((halfedge->origin->point.y > point.y) != (halfedge->destination->point.y > point.y))
             && (point.x < (halfedge->destination->point.x - halfedge->origin->point.x)
                * (point.y - halfedge->origin->point.y) / (halfedge->destination->point.y - halfedge->origin->point.y) + halfedge->origin->point.x)){
            in = !in;
        }
        halfedge = halfedge->next;

    } while (halfedge != seed->face->outerComponent);

    return in;
}

// template<typename T>
// bool inObstacle(Box<T> ObstacleBox, Vector2<T> point)
// {
//     bool in = ObstacleBox.contains(point);
//     return in;
// }

template<typename T>
bool inObstacles(std::vector<Box<T>> ObstacleBoxes, Vector2<T> point)
{
    // std::cout << "------------- DEBUG: -------------- \n";
    // std::cout << "x: " << point.x << ", y: " << point.y << std::endl;

    bool in = false;
    std::vector<bool> boolVector(ObstacleBoxes.size());      // vettore di bool con stessa dimensione di ObstacleBoxes
    for (int i=0; i<ObstacleBoxes.size(); ++i)
    {
        // std::cout << "---- Obstacle ----  \n";
        // std::cout << "bottom left: " << ObstacleBoxes[i].left << ObstacleBoxes[i].bottom << std::endl;
        // std::cout << "top right: " << ObstacleBoxes[i].right << ObstacleBoxes[i].top << std::endl;
        boolVector[i] = ObstacleBoxes[i].contains(point);
    }
    
    // in = false solo se tutti false
    for (int i=0; i<boolVector.size(); ++i)
    {
        in = in || boolVector[i];
    }

    return in;
}

//Funzione creata per obiettivi di debug nel calcolo dell'area
template<typename T>
double computePolygonArea(const Diagram<T> &polygon, Vector2<T> pt_mean, T var, double discretize_precision = 1.0/100.0){
    //Calcolo p_t in coordinate locali (local_p_t);
    auto local_pt_mean = pt_mean - polygon.getGlobalPoint();

    auto seed = polygon.getSite(0);
    auto halfedge = seed->face->outerComponent;

    //trova gli estremi del rettangolo che contengono il poligono
    double x_inf = std::min(halfedge->origin->point.x, halfedge->destination->point.x);
    double x_sup = std::max(halfedge->origin->point.x, halfedge->destination->point.x);
    double y_inf = std::min(halfedge->origin->point.y, halfedge->destination->point.y);
    double y_sup = std::max(halfedge->origin->point.y, halfedge->destination->point.y);
    halfedge = halfedge->next;

    //DEBUG
    std::vector<double> debug_x;
    std::vector<double> debug_y;
    debug_x.push_back(halfedge->origin->point.x);
    debug_y.push_back(halfedge->origin->point.y);
    debug_x.push_back(halfedge->destination->point.x);
    debug_y.push_back(halfedge->destination->point.y);

    do{
        //------------------ x component --------------------
        if (x_inf > halfedge->destination->point.x)
        {
            x_inf = halfedge->destination->point.x;

        } else if (x_sup < halfedge->destination->point.x)
        {
            x_sup = halfedge->destination->point.x;
        }

        //------------------ y component --------------------
        if (y_inf > halfedge->destination->point.y)
        {
            y_inf = halfedge->destination->point.y;
        } else if (y_sup < halfedge->destination->point.y)
        {
            y_sup = halfedge->destination->point.y;
        }

        halfedge = halfedge->next;
        //DEBUG
        debug_x.push_back(halfedge->destination->point.x);
        debug_y.push_back(halfedge->destination->point.y);

    } while (halfedge != seed->face->outerComponent);


    //DEBUG
    /*
    std::cout<<"debug vectors x: "<<std::scientific;
    for (int i = 0; i < debug_x.size(); ++i)
    {
        std::cout<<debug_x[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;

    std::cout<<"debug vectors y: "<<std::scientific;
    for (int i = 0; i < debug_y.size(); ++i)
    {
        std::cout<<debug_y[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;
    */

    double dx = (x_sup - x_inf)/2.0 * discretize_precision;
    double dy = (y_sup - y_inf)/2.0 * discretize_precision;
    double dA = dx*dy;
    double A = 0;

    for (double i = x_inf; i <= x_sup; i=i+dx)
    {
        for (double j = y_inf; j <= y_sup; j=j+dy)
        {
            //std::cout<<"j value :: "<<j<<"\n"<<std::scientific;
            bool in = inPolygon(polygon, Vector2<double> {i+dx, j+dy});
            if (in)
            {
                A = A + dA*gauss_pdf(local_pt_mean, var, Vector2<double> {i,j});
            }
        }
    }
    return A;
}

// Funzione per calcolare il centroide senza gaussiane
template<typename T>
Vector2<T> calculateCentroid(const Diagram<T> &polygon, std::vector<Box<T>> ObstacleBoxes, double discretize_precision = 1.0/100.0){

    auto seed = polygon.getSite(0);
    auto halfedge = seed->face->outerComponent;

    //trova gli estremi del rettangolo che contengono il poligono
    double x_inf = std::min(halfedge->origin->point.x, halfedge->destination->point.x);
    double x_sup = std::max(halfedge->origin->point.x, halfedge->destination->point.x);
    double y_inf = std::min(halfedge->origin->point.y, halfedge->destination->point.y);
    double y_sup = std::max(halfedge->origin->point.y, halfedge->destination->point.y);
    halfedge = halfedge->next;

    //DEBUG
    std::vector<double> debug_x;
    std::vector<double> debug_y;
    debug_x.push_back(halfedge->origin->point.x);
    debug_y.push_back(halfedge->origin->point.y);
    debug_x.push_back(halfedge->destination->point.x);
    debug_y.push_back(halfedge->destination->point.y);

    do{
        //------------------ x component --------------------
        if (x_inf > halfedge->destination->point.x)
        {
            x_inf = halfedge->destination->point.x;

        } else if (x_sup < halfedge->destination->point.x)
        {
            x_sup = halfedge->destination->point.x;
        }

        //------------------ y component --------------------
        if (y_inf > halfedge->destination->point.y)
        {
            y_inf = halfedge->destination->point.y;
        } else if (y_sup < halfedge->destination->point.y)
        {
            y_sup = halfedge->destination->point.y;
        }

        halfedge = halfedge->next;
        //DEBUG
        debug_x.push_back(halfedge->destination->point.x);
        debug_y.push_back(halfedge->destination->point.y);

    } while (halfedge != seed->face->outerComponent);

    double dx = (x_sup - x_inf)/2.0 * discretize_precision;
    double dy = (y_sup - y_inf)/2.0 * discretize_precision;
    double dA = dx*dy;
    double A = 0;
    double Cx = 0, Cy = 0;

    std::cout << "Coordinate sito: " << polygon.getGlobalPoint() << std::endl;

    // Rielaborazione ostacoli in coord locali
    std::vector<Box<double>> localObstacleBoxes(ObstacleBoxes.size());
    for (int i=0; i<ObstacleBoxes.size(); ++i)
    {
        localObstacleBoxes[i] = reworkObstacle(ObstacleBoxes[i], polygon.getGlobalPoint());
    }

    for (double i = x_inf; i <= x_sup; i=i+dx)
    {
        for (double j = y_inf; j <= y_sup; j=j+dy)
        {   
            bool inArea = inPolygon(polygon, Vector2<double> {i+dx, j+dy});
            bool inObstacle = inObstacles(localObstacleBoxes, Vector2<double> {i+dx, j+dy});
            if (inArea && !inObstacle)
            {
                A = A + dA;
                Cx = Cx + i*dA;
                Cy = Cy + j*dA;
            }
        }
    }
    Cx = Cx / A;
    Cy = Cy / A;

    if (DEBUG >= 1)
    {
        std::cout<<std::scientific<<" ------------------------ Area : "<<A<<std::endl;
    }

    Vector2<double> C = {Cx, Cy};
    
    return C;
}

//Calcolo del centroide discretizzando l'area del poligono
//viene creato un rettangolo circoscritto al poligono e suddiviso in tanti elementi quanti definito da parametro
//ogni elemento viene considerato se appartenente al poligono (inPolygon)
//per ogni elemento viene calcolato il valore della PDF nel suo baricentro
template<typename T>
Vector2<T> computePolygonCentroid(const Diagram<T> &polygon, std::vector<Vector2<T>> pt_means, std::vector<T> vars, double discretize_precision = 1.0/100.0){
    //Calcolo p_t in coordinate locali (local_p_t);
    for (long unsigned int i = 0; i < pt_means.size(); ++i)
    {
        pt_means[i] = pt_means[i] - polygon.getGlobalPoint();
        //auto local_pt_mean = pt_mean - polygon.getGlobalPoint();
    }

    //DEBUG
    //std::cout<<"gaussian relative position ::: "<<local_pt_mean<<"\n"<<std::scientific;

    auto seed = polygon.getSite(0);
    auto halfedge = seed->face->outerComponent;

    //trova gli estremi del rettangolo che contengono il poligono
    double x_inf = std::min(halfedge->origin->point.x, halfedge->destination->point.x);
    double x_sup = std::max(halfedge->origin->point.x, halfedge->destination->point.x);
    double y_inf = std::min(halfedge->origin->point.y, halfedge->destination->point.y);
    double y_sup = std::max(halfedge->origin->point.y, halfedge->destination->point.y);
    halfedge = halfedge->next;

    //DEBUG
    std::vector<double> debug_x;
    std::vector<double> debug_y;
    debug_x.push_back(halfedge->origin->point.x);
    debug_y.push_back(halfedge->origin->point.y);
    debug_x.push_back(halfedge->destination->point.x);
    debug_y.push_back(halfedge->destination->point.y);

    do{
        //------------------ x component --------------------
        if (x_inf > halfedge->destination->point.x)
        {
            x_inf = halfedge->destination->point.x;

        } else if (x_sup < halfedge->destination->point.x)
        {
            x_sup = halfedge->destination->point.x;
        }

        //------------------ y component --------------------
        if (y_inf > halfedge->destination->point.y)
        {
            y_inf = halfedge->destination->point.y;
        } else if (y_sup < halfedge->destination->point.y)
        {
            y_sup = halfedge->destination->point.y;
        }

        halfedge = halfedge->next;
        //DEBUG
        debug_x.push_back(halfedge->destination->point.x);
        debug_y.push_back(halfedge->destination->point.y);

    } while (halfedge != seed->face->outerComponent);

    //DEBUG
    /*
    std::cout<<"debug vectors x: "<<std::scientific;
    for (int i = 0; i < debug_x.size(); ++i)
    {
        std::cout<<debug_x[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;

    std::cout<<"debug vectors y: "<<std::scientific;
    for (int i = 0; i < debug_y.size(); ++i)
    {
        std::cout<<debug_y[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;
    */

    double dx = (x_sup - x_inf)/2.0 * discretize_precision;
    double dy = (y_sup - y_inf)/2.0 * discretize_precision;
    double dA = dx*dy;
    double A = 0;
    double Cx = 0, Cy = 0;

    for (double i = x_inf; i <= x_sup; i=i+dx)
    {
        for (double j = y_inf; j <= y_sup; j=j+dy)
        {
            //std::cout<<"j value :: "<<j<<"\n"<<std::scientific;
            bool in = inPolygon(polygon, Vector2<double> {i+dx, j+dy});
            if (in)
            {
                double dA_pdf;
                if (vars.size() <= 1)
                {
                    dA_pdf = dA*gauss_pdf_std(pt_means[0], vars[0], Vector2<double> {i,j});
                } else {
                    dA_pdf = dA*multiple_gauss_pdf(pt_means, vars, Vector2<double> {i,j});
                }
                A = A + dA_pdf;
                Cx = Cx + i*dA_pdf;
                Cy = Cy + j*dA_pdf;
            }
        }
    }
    Cx = Cx / A;
    Cy = Cy / A;

    if (DEBUG >= 1)
    {
        std::cout<<std::scientific<<" ------------------------ Area : "<<A<<std::endl;
    }

    Vector2<double> C = {Cx, Cy};
    return C;
}

// Funzione per calcolare il centroide con distribuzione gaussiana repulsiva
template<typename T>
Vector2<T> computeRepulsivePolygonCentroid(const Diagram<T> &polygon, std::vector<Vector2<T>> pt_means, std::vector<T> vars, std::vector<Box<T>> ObstacleBoxes, std::vector<T> &k_vect, std::vector<bool> fixed, double discretize_precision = 1.0/100.0){
    //Calcolo p_t in coordinate locali (local_p_t);
    for (long unsigned int i = 0; i < pt_means.size(); ++i)
    {
        pt_means[i] = pt_means[i] - polygon.getGlobalPoint();
        //auto local_pt_mean = pt_mean - polygon.getGlobalPoint();
    }

    //DEBUG
    //std::cout<<"gaussian relative position ::: "<<local_pt_mean<<"\n"<<std::scientific;

    auto seed = polygon.getSite(0);
    auto halfedge = seed->face->outerComponent;

    //trova gli estremi del rettangolo che contengono il poligono
    double x_inf = std::min(halfedge->origin->point.x, halfedge->destination->point.x);
    double x_sup = std::max(halfedge->origin->point.x, halfedge->destination->point.x);
    double y_inf = std::min(halfedge->origin->point.y, halfedge->destination->point.y);
    double y_sup = std::max(halfedge->origin->point.y, halfedge->destination->point.y);
    halfedge = halfedge->next;

    //DEBUG
    std::vector<double> debug_x;
    std::vector<double> debug_y;
    debug_x.push_back(halfedge->origin->point.x);
    debug_y.push_back(halfedge->origin->point.y);
    debug_x.push_back(halfedge->destination->point.x);
    debug_y.push_back(halfedge->destination->point.y);

    do{
        //------------------ x component --------------------
        if (x_inf > halfedge->destination->point.x)
        {
            x_inf = halfedge->destination->point.x;

        } else if (x_sup < halfedge->destination->point.x)
        {
            x_sup = halfedge->destination->point.x;
        }

        //------------------ y component --------------------
        if (y_inf > halfedge->destination->point.y)
        {
            y_inf = halfedge->destination->point.y;
        } else if (y_sup < halfedge->destination->point.y)
        {
            y_sup = halfedge->destination->point.y;
        }

        halfedge = halfedge->next;
        //DEBUG
        debug_x.push_back(halfedge->destination->point.x);
        debug_y.push_back(halfedge->destination->point.y);

    } while (halfedge != seed->face->outerComponent);

    //DEBUG
    /*
    std::cout<<"debug vectors x: "<<std::scientific;
    for (int i = 0; i < debug_x.size(); ++i)
    {
        std::cout<<debug_x[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;

    std::cout<<"debug vectors y: "<<std::scientific;
    for (int i = 0; i < debug_y.size(); ++i)
    {
        std::cout<<debug_y[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;
    */

    double dx = (x_sup - x_inf)/2.0 * discretize_precision;
    double dy = (y_sup - y_inf)/2.0 * discretize_precision;
    double dA = dx*dy;
    double A = 0;
    double Cx = 0, Cy = 0;

    std::cout << "Coordinate sito: " << polygon.getGlobalPoint() << std::endl;

    // Rielaborazione ostacoli in coord locali
    std::vector<Box<double>> localObstacleBoxes(ObstacleBoxes.size());
    for (int i=0; i<ObstacleBoxes.size(); ++i)
    {
        localObstacleBoxes[i] = reworkObstacle(ObstacleBoxes[i], polygon.getGlobalPoint());
    }

    // std::cout << "======= Coordinate locali ostacoli: ========\n";
    // for (int i=0; i<localObstacleBoxes.size(); i++)
    // {
    //     std::cout << "Ostacolo num. " << i << std::endl;
    //     std::cout << localObstacleBoxes[i].left << ", " << localObstacleBoxes[i].bottom << std::endl;
    //     std::cout << localObstacleBoxes[i].right << ", " << localObstacleBoxes[i].bottom << std::endl;
    //     std::cout << localObstacleBoxes[i].right << ", " << localObstacleBoxes[i].top << std::endl;
    //     std::cout << localObstacleBoxes[i].left << ", " << localObstacleBoxes[i].top << std::endl;
    // }

    // Cerco gaussiane visibili
    std::vector<Vector2<T>> visible_gauss;
    std::vector<T> visible_vars;
    std::vector<T> visible_k;
    std::vector<bool> visible_fixed;
    
    // Controllo se la gaussiana e nascosta dietro a un ostacolo (guardo se il segmento che congiunge il robot con la gaussiana interseca almeno un lato dell'ostacolo)
    for (int i=0; i<pt_means.size(); i++)
    {
        // bool doIntersect = false;
        // Vector2<T> global_mean{};
        // global_mean.x = (pt_means[i].x) + (polygon.getGlobalPoint().x);
        // global_mean.y = (pt_means[i].y) + (polygon.getGlobalPoint().y);
        // for (int j=0; j<localObstacleBoxes.size(); j++)
        // {
        //     // Cerco intersezione con singolo ostacolo
        //     doIntersect = doIntersect || intersectLineAndObstacle(seed->point, pt_means[i], localObstacleBoxes[j]);
        // }
        double dist = std::sqrt(std::pow((pt_means[i].x-seed->point.x),2) + std::pow((pt_means[i].y-seed->point.y),2));
        // Se il segmento robot-gaussiana non interseca nessun ostacolo tengo conto del peso di quella gaussiana, altrimenti no
        if (dist < 4)                               // d < sensing range
        {
            visible_gauss.push_back(pt_means[i]);
            visible_vars.push_back(vars[i]);
            visible_k.push_back(k_vect[i]);
            visible_fixed.push_back(fixed[i]);
        } 
        // else
        // {
        //     std::cout << "Intersection found\n";
        // }
    }
    // std::cout << "Sito: " << seed->point << std::endl;
    // std::cout << "Visible gaussians: \n";
    // for (int j=0; j<visible_gauss.size(); j++)
    // {
    //     // std::cout << "==================\n";
    //     std::cout << visible_gauss[j] << std::endl;
    //     // std::cout << visible_vars[j] << std::endl;
    //     // std::cout << visible_k[j] << std::endl;
    //     // std::cout << visible_fixed[j] << std::endl;
    //     // std::cout << "==================\n";
    // }

    if (visible_vars.size() < 1)
    {
        Vector2<double> Centr = calculateCentroid(polygon, ObstacleBoxes, discretize_precision);
        Cx = Centr.x;
        Cy = Centr.y;
    } else
    {
        for (double i = x_inf; i <= x_sup; i=i+dx)
        {
            for (double j = y_inf; j <= y_sup; j=j+dy)
            {   
                bool inArea = inPolygon(polygon, Vector2<double> {i+dx, j+dy});
                bool inObstacle = inObstacles(localObstacleBoxes, Vector2<double> {i+dx, j+dy});
                if (inArea && !inObstacle)
                {
                    double dA_pdf;
                    if (vars.size() <= 1)
                    {
                        dA_pdf = dA*gauss_pdf(visible_gauss[0], visible_vars[0], Vector2<double> {i,j}, visible_k[0], visible_fixed[0]);
                    } else {
                        dA_pdf = dA*multiple_visible_gauss_pdf(visible_gauss, visible_vars, Vector2<double> {i,j}, visible_k, visible_fixed);
                    }
                    // dA_pdf = dA*multiple_visible_gauss_pdf(visible_gauss, visible_vars, Vector2<double> {i,j}, visible_k, visible_fixed);
                    A = A + dA_pdf;
                    Cx = Cx + i*dA_pdf;
                    Cy = Cy + j*dA_pdf;
                }
            }
        }
        Cx = Cx / A;
        Cy = Cy / A;
    }

    if (DEBUG >= 1)
    {
        std::cout<<std::scientific<<" ------------------------ Area : "<<A<<std::endl;
    }

    Vector2<double> C = {Cx, Cy};
    
    return C;
}

// Funzione per il calcolo del centroide in caso di GMM
template<typename T>
std::vector<float> computeGMMPolygonCentroid(const Diagram<double> &polygon, std::vector<std::vector<T>> pt_means, std::vector<std::vector<std::vector<T>>> vars, std::vector<T> weights, float discretize_precision = 1.0/10.0){
    //Calcolo p_t in coordinate locali (local_p_t);
    for (long unsigned int i = 0; i < pt_means.size(); ++i)
    {
        pt_means[i][0] = pt_means[i][0] - polygon.getGlobalPoint().x;
        pt_means[i][1] = pt_means[i][1] - polygon.getGlobalPoint().y;
    }
    // std::cout << "SONO QUI 1\n";

    //DEBUG
    // std::cout<<"gaussian relative position ::: "<<pt_means[0]<<"\n"<<std::scientific;
    // std::cout << "Elenco gaussiane relative: \n";
    // for (int i=0; i<pt_means.size(); i++)
    // {
    //     std::cout << pt_means[i][0] << ", " << pt_means[i][1] << std::endl;
    // }

    auto seed = polygon.getSite(0);
    auto halfedge = seed->face->outerComponent;

    //trova gli estremi del rettangolo che contengono il poligono
    float x_inf = std::min(halfedge->origin->point.x, halfedge->destination->point.x);
    float x_sup = std::max(halfedge->origin->point.x, halfedge->destination->point.x);
    float y_inf = std::min(halfedge->origin->point.y, halfedge->destination->point.y);
    float y_sup = std::max(halfedge->origin->point.y, halfedge->destination->point.y);
    float z_inf = -2.0; float z_sup = 2.0;
    halfedge = halfedge->next;


    //DEBUG
    std::vector<double> debug_x;
    std::vector<double> debug_y;
    debug_x.push_back(halfedge->origin->point.x);
    debug_y.push_back(halfedge->origin->point.y);
    debug_x.push_back(halfedge->destination->point.x);
    debug_y.push_back(halfedge->destination->point.y);

    do{
        //------------------ x component --------------------
        if (x_inf > halfedge->destination->point.x)
        {
            x_inf = halfedge->destination->point.x;

        } else if (x_sup < halfedge->destination->point.x)
        {
            x_sup = halfedge->destination->point.x;
        }

        //------------------ y component --------------------
        if (y_inf > halfedge->destination->point.y)
        {
            y_inf = halfedge->destination->point.y;
        } else if (y_sup < halfedge->destination->point.y)
        {
            y_sup = halfedge->destination->point.y;
        }

        halfedge = halfedge->next;
        //DEBUG
        debug_x.push_back(halfedge->destination->point.x);
        debug_y.push_back(halfedge->destination->point.y);

    } while (halfedge != seed->face->outerComponent);


    std::cout << "x_inf : " << x_inf << " x_sup : " << x_sup << " y_inf : " << y_inf << " y_sup : " << y_sup << std::endl;

    //DEBUG
    /*
    std::cout<<"debug vectors x: "<<std::scientific;
    for (int i = 0; i < debug_x.size(); ++i)
    {
        std::cout<<debug_x[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;

    std::cout<<"debug vectors y: "<<std::scientific;
    for (int i = 0; i < debug_y.size(); ++i)
    {
        std::cout<<debug_y[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;
    */

    float dx = (x_sup - x_inf)/2.0 * discretize_precision;
    float dy = (y_sup - y_inf)/2.0 * discretize_precision;
    float dz = (z_sup - z_inf)/2.0 * discretize_precision;
    float dV = dx*dy*dz;
    float V = 0;
    float Cx = 0, Cy = 0, Cz = 0;

    // std::cout << "dx: " << dx << " dy: " << dy << " dz: " << dz << " dV: " << dV << std::endl;

    // for (float i = (float)x_inf; i <= x_sup; i=i+dx)
    // {
    //     for (float j = (float)y_inf; j <= y_sup; j=j+dy)
    //     {
    //         for (float k = (float)z_inf; k<=z_sup; k=k+dz)
    //         {
    //             //std::cout<<"j value :: "<<j<<"\n"<<std::scientific;
    //             bool in = inPolygon(polygon, Vector2<double> {i+dx, j+dy});
    //             if (in)
    //             {
    //                 std::vector<float> point = {i, j};
    //                 float dV_pdf;
    //                 if (vars.size() <= 1)
    //                 {
    //                     dV_pdf = dV*gauss3d_pdf(pt_means[0], vars[0], point);
    //                 } else {
    //                     dV_pdf = dV*multiple_gauss3d_pdf(pt_means, vars, point, weights);
    //                 }
    //                 V = V + dV_pdf;
    //                 Cx = Cx + i*dV_pdf;
    //                 Cy = Cy + j*dV_pdf;
    //                 Cz = Cz + k*dV_pdf;
    //                 // std::cout << "Valori provvisori: " << Cx << " " << Cy << " " << Cz << " " << V << std::endl;
    //             }
    //         }
    //     }
    // }
    // Cx = Cx / V;
    // Cy = Cy / V;
    // Cz = Cz / V;

    for (float i = (float)x_inf; i <= x_sup; i=i+dx)
    {
        for (float j = (float)y_inf; j <= y_sup; j=j+dy)
        {
            //std::cout<<"j value :: "<<j<<"\n"<<std::scientific;
            bool in = inPolygon(polygon, Vector2<double> {i+dx, j+dy});
            if (in)
            {
                std::vector<float> point = {i, j};
                float dV_pdf;
                if (vars.size() <= 1)
                {
                    dV_pdf = dV*gauss3d_pdf(pt_means[0], vars[0], point);
                } else {
                    dV_pdf = dV*multiple_gauss3d_pdf(pt_means, vars, point, weights);
                }
                V = V + dV_pdf;
                Cx = Cx + i*dV_pdf;
                Cy = Cy + j*dV_pdf;
                // std::cout << "Valori provvisori: " << Cx << " " << Cy << " " << Cz << " " << V << std::endl;
            }
        }
    }
    Cx = Cx / V;
    Cy = Cy / V;
    // Cz = Cz / V;

    // if (DEBUG >= 1)
    // {
    //     std::cout<<std::scientific<<" ------------------------ Area : "<<A<<std::endl;
    // }

    std::vector<float> C = {Cx, Cy, Cz};
    return C;
}

// Funzione per il calcolo del centroide in caso di GMM definite con messaggio custom
std::vector<float> computeGMMPolygonCentroid2(const Diagram<double> &polygon, turtlebot3_msgs::msg::GMM gmm, std::vector<Box<double>> ObstacleBoxes = {}, float discretize_precision = 1.0/10.0){
    
    // Transform GMM to local coordinates
    turtlebot3_msgs::msg::GMM gmm_local;
    gmm_local.weights = gmm.weights;
    for (long unsigned int i = 0; i < gmm.gaussians.size(); ++i)
    {
        gmm_local.gaussians.push_back(gmm.gaussians[i]);
        gmm_local.gaussians[i].mean_point.x = gmm.gaussians[i].mean_point.x - polygon.getGlobalPoint().x;
        gmm_local.gaussians[i].mean_point.y = gmm.gaussians[i].mean_point.y - polygon.getGlobalPoint().y;
        gmm_local.gaussians[i].mean_point.z = 0.0;
        gmm_local.gaussians[i].covariance = gmm.gaussians[i].covariance;
    }

    //DEBUG
    // std::cout<<"gaussian relative position ::: "<<pt_means[0]<<"\n"<<std::scientific;
    // std::cout << "Elenco gaussiane relative: \n";
    // for (int i=0; i<gmm_local.gaussians.size(); i++)
    // {
    //     std::cout << gmm_local.gaussians[i].mean_point.x << ", " << gmm_local.gaussians[i].mean_point.y << std::endl;
    // }

    auto seed = polygon.getSite(0);
    auto halfedge = seed->face->outerComponent;

    //trova gli estremi del rettangolo che contengono il poligono
    float x_inf = std::min(halfedge->origin->point.x, halfedge->destination->point.x);
    float x_sup = std::max(halfedge->origin->point.x, halfedge->destination->point.x);
    float y_inf = std::min(halfedge->origin->point.y, halfedge->destination->point.y);
    float y_sup = std::max(halfedge->origin->point.y, halfedge->destination->point.y);
    float z_inf = -2.0; float z_sup = 2.0;
    halfedge = halfedge->next;


    //DEBUG
    std::vector<double> debug_x;
    std::vector<double> debug_y;
    debug_x.push_back(halfedge->origin->point.x);
    debug_y.push_back(halfedge->origin->point.y);
    debug_x.push_back(halfedge->destination->point.x);
    debug_y.push_back(halfedge->destination->point.y);

    do{
        //------------------ x component --------------------
        if (x_inf > halfedge->destination->point.x)
        {
            x_inf = halfedge->destination->point.x;

        } else if (x_sup < halfedge->destination->point.x)
        {
            x_sup = halfedge->destination->point.x;
        }

        //------------------ y component --------------------
        if (y_inf > halfedge->destination->point.y)
        {
            y_inf = halfedge->destination->point.y;
        } else if (y_sup < halfedge->destination->point.y)
        {
            y_sup = halfedge->destination->point.y;
        }

        halfedge = halfedge->next;
        //DEBUG
        debug_x.push_back(halfedge->destination->point.x);
        debug_y.push_back(halfedge->destination->point.y);

    } while (halfedge != seed->face->outerComponent);


    std::cout << "x_inf : " << x_inf << " x_sup : " << x_sup << " y_inf : " << y_inf << " y_sup : " << y_sup << std::endl;

    //DEBUG
    /*
    std::cout<<"debug vectors x: "<<std::scientific;
    for (int i = 0; i < debug_x.size(); ++i)
    {
        std::cout<<debug_x[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;

    std::cout<<"debug vectors y: "<<std::scientific;
    for (int i = 0; i < debug_y.size(); ++i)
    {
        std::cout<<debug_y[i]<<" "<<std::scientific;
    }
    std::cout<<"\n"<<std::scientific;
    */

    float dx = (x_sup - x_inf)/2.0 * discretize_precision;
    float dy = (y_sup - y_inf)/2.0 * discretize_precision;
    float dz = (z_sup - z_inf)/2.0 * discretize_precision;
    float dV = dx*dy*dz;
    float V = 0;
    float Cx = 0, Cy = 0, Cz = 0;

    // Transform obstacles to local coordinates
    std::vector<Box<double>> localObstacleBoxes(ObstacleBoxes.size());
    for (int i=0; i<ObstacleBoxes.size(); ++i)
    {
        localObstacleBoxes[i] = reworkObstacle(ObstacleBoxes[i], polygon.getGlobalPoint());
    }

    for (float i = (float)x_inf; i <= x_sup; i=i+dx)
    {
        for (float j = (float)y_inf; j <= y_sup; j=j+dy)
        {
            //std::cout<<"j value :: "<<j<<"\n"<<std::scientific;
            bool inArea = inPolygon(polygon, Vector2<double> {i+dx, j+dy});
            bool inObstacle = inObstacles(localObstacleBoxes, Vector2<double> {i+dx, j+dy});
            if (inArea && !inObstacle)
            {
                std::vector<float> point = {i, j};
                float dV_pdf;
                if (gmm_local.gaussians.size() <= 1)
                {
                    dV_pdf = dV*single_component_pdf(gmm_local.gaussians[0], point);
                } else {
                    dV_pdf = dV*mixture_pdf(gmm_local, point);
                }
                V = V + dV_pdf;
                Cx = Cx + i*dV_pdf;
                Cy = Cy + j*dV_pdf;
            }
        }
    }
    Cx = Cx / V;
    Cy = Cy / V;
    // Cz = Cz / V;

    // if (DEBUG >= 1)
    // {
    //     std::cout<<std::scientific<<" ------------------------ Area : "<<A<<std::endl;
    // }

    std::vector<float> C = {Cx, Cy, Cz};
    return C;
}

template<typename T>
std::vector<Vector2<T>> computeDiagramsCentroids(const std::vector<Diagram<T>>& diagrams, std::vector<Vector2<T>> pt_means, std::vector<T> vars, double discretize_precision = 1.0/100.0){
    //Vettore dei centroidi gaussiani
    std::vector<Vector2<T>> centroids;

    for (const auto &diagram : diagrams)
    {
        auto C = computePolygonCentroid(diagram, pt_means, vars, discretize_precision);
        centroids.push_back(C);
    }
    return centroids;
}

template<typename T>
std::vector<Vector2<T>> computeRepulsiveDiagramsCentroids(const std::vector<Diagram<T>>& diagrams, std::vector<Vector2<T>> pt_means, std::vector<T> vars, std::vector<Box<T>> ObstacleBoxes, std::vector<T> &k_vect, std::vector<bool> fixed, double discretize_precision = 1.0/100.0){
    //Vettore dei centroidi gaussiani
    std::vector<Vector2<T>> centroids;

    for (const auto &diagram : diagrams)
    {
        Vector2<T> C = {0.0, 0.0};
        if (vars.size() < 1)
        {
            // se non ci sono gaussiane devo usare una funzione diversa
            C = calculateCentroid(diagram, ObstacleBoxes, discretize_precision);
        } else
        {
            C = computeRepulsivePolygonCentroid(diagram, pt_means, vars, ObstacleBoxes, k_vect, fixed, discretize_precision);
        }
        centroids.push_back(C);
    }
    return centroids;
}

template<typename T>
std::vector<std::vector<T>> computeGMMDiagramsCentroids(const std::vector<Diagram<T>>& diagrams, std::vector<std::vector<T>> pt_means, std::vector<std::vector<std::vector<T>>> vars, std::vector<T> weights,  double discretize_precision = 1.0/10.0){
    //Vettore dei centroidi gaussiani
    std::vector<std::vector<T>> centroids;

    for (const auto &diagram : diagrams)
    {
        std::vector<T> C = {0.0, 0.0, 0.0};
        C = computeGMMPolygonCentroid(diagram, pt_means, vars, weights, discretize_precision);
    }
    return centroids;
}



//funzione per calcolare il valore di un punto data una distribuzione a forma di gaussiana
template<typename T>
double gauss_pdf_std(Vector2<T> mean_pt, T var, Vector2<T> pt){
    double temp = (std::pow((pt.x-mean_pt.x),2.0) + std::pow((pt.y-mean_pt.y),2.0)) / (2.0 * std::pow(var,2.0));
    return std::exp(-temp);
}

//funzione per calcolare il valore di un punto data una distribuzione a forma di gaussiana
// Calcola valore positivo senza ribaltarla, quindi taglio i valori maggiori di k
template<typename T>
double gauss_pdf(Vector2<T> mean_pt, T var, Vector2<T> pt, T &k, bool fixed){
    double val = 0;
    double temp = (std::pow((pt.x-mean_pt.x),2.0) + std::pow((pt.y-mean_pt.y),2.0)) / (2.0 * std::pow(var,2.0));
    double temp2 = std::exp(-temp);
    if (temp2 > k)
    {
        val = k;
    } else
    {
        val = temp2;
    }
    return val;
}

float gauss3d_pdf(std::vector<float> mean, std::vector<std::vector<float>> var, std::vector<float> pt){
    // Check dimensions
    int d = mean.size();
    int d_var = var.size()*var[0].size();
    int d_pt = pt.size();
    
    if (d*d != d_var || d != d_pt)
    {
        std::cout<<"Error in the GMM definition: dimension mismatch"<<std::endl;
        std::cout << "mean dimension: " << d << std::endl;
        std::cout << "var dimension: " << d_var << std::endl;
        std::cout << "pt dimension:" << d_pt << std::endl;
        return -1;
    }

    if (d > 3 || d_var > 9 || d_pt > 3)
    {
        std::cout<<"Error in the GMM definition: excessive dimension"<<std::endl;
        return -1;
    }

    float det = getDeterminant(var);                          // determinant
    if (det <= 0)
    {
        std::cout << "Error! Negative determinant: " << det << std::endl;; 
        return -1;
    }
    float denom = std::sqrt(std::pow(2*M_PI,d)*det);       // denominator

    // Create elements of the equation
    std::vector<std::vector<float>> row_matrix = {{}};                                                  // difference between considered point and mean
    for (int i = 0; i < d; i++)
    {
        row_matrix[0].push_back(pt[i]-mean[i]);
    }


    std::vector<std::vector<float>> inv_cov = getInverse(var);                       
    std::vector<std::vector<float>> temp = multiplyMatrices(row_matrix, inv_cov);                               // first step pseudo-inverse calculation
    std::vector<std::vector<float>> temp2 = multiplyMatrices(temp,getTranspose(row_matrix));                    // end pseudo-inverse calculation
    float prob = 1/denom*std::exp(-0.5*temp2[0][0]);                                                            // final prob value

    return prob;
}

float gauss3d_pdf2(turtlebot3_msgs::msg::Gaussian gaussian, std::vector<float> pt){
    std::vector<float> mean = {};                                // mean vector (1D, 2D or 3D)
    std::vector<std::vector<float>> var = {};                    // covariance matrix
    std::vector<float> row = {};                                 // covariance matrix rows
    int var_size = sqrt(gaussian.covariance.size());             // covariance matrix size: var_size * var_size

    if (var_size == 1)
    {
        row.push_back(gaussian.covariance[0]);
        var.push_back(row);
        mean.push_back(gaussian.mean_point.x);
    } else if (var_size == 2)
    {
        for (int i=0; i<var_size+1; i=i+2)
        {
            row.clear();
            row.push_back(gaussian.covariance[i]);
            row.push_back(gaussian.covariance[i+1]);
            var.push_back(row);
        }
        
        mean.push_back(gaussian.mean_point.x);
        mean.push_back(gaussian.mean_point.y);

    } else if (var_size == 3)
    {
        for (int i = 0; i < var_size+1; i=i+3)
            {
                row.clear();
                row.push_back(gaussian.covariance[i]);
                row.push_back(gaussian.covariance[i+1]);
                row.push_back(gaussian.covariance[i+2]);
                var.push_back(row);
            }
        mean.push_back(gaussian.mean_point.x);
        mean.push_back(gaussian.mean_point.y);
        mean.push_back(gaussian.mean_point.z);

    } else {
        std::cout << "Error in the GMM definition: Dimention mismatch" << std::endl;
        std::cout << "Mean point dimension: " << mean.size() << std::endl;
        std::cout << "Covariance matrix dimension: " << var_size << " x " << var_size << std::endl;
        return -1;
    }

    int d = mean.size();                                    // mean point dimension

    float det = getDeterminant(var);                          // determinant
    if (det <= 0)
    {  
        std::cout << "Error! Negative determinant: " << det << std::endl;
        return -1;
    }
    float denom = std::sqrt(std::pow(2*M_PI,d)*det);       // denominator

    // Create elements of the equation
    std::vector<std::vector<float>> row_matrix = {{}};                                // difference between considered point and mean
    for (int i = 0; i < d; i++)
    {
        row_matrix[0].push_back(pt[i]-mean[i]);
    }


    std::vector<std::vector<float>> inv_cov = getInverse(var);                        // covariance matrix inverse
    std::vector<std::vector<float>> temp = multiplyMatrices(row_matrix, inv_cov);                  // first step pseudo-inverse calculation
    std::vector<std::vector<float>> temp2 = multiplyMatrices(temp,getTranspose(row_matrix));                     // end of pseudo-inverse calculation
    float prob = 1/denom*std::exp(-0.5*temp2[0][0]);                                     // final prob value

    return prob;
}


float single_component_pdf(turtlebot3_msgs::msg::Gaussian gaussian, std::vector<float> pt){
    // std::cout << "Entered single component pdf" << std::endl;
    Eigen::VectorXd mean_pt;
    Eigen::MatrixXd cov_matrix;
    int size = sqrt(gaussian.covariance.size());        // gaussian covariance matrix size: var_size * var_size
    mean_pt.resize(size);
    cov_matrix.resize(size, size);

    // std::cout << "Size: " << size << std::endl;

    if (size == 1)
    {
        mean_pt(0) = gaussian.mean_point.x;
        cov_matrix(0,0) = gaussian.covariance[0];
    } else if (size == 2)
    {
        mean_pt(0) = gaussian.mean_point.x;
        mean_pt(1) = gaussian.mean_point.y;
        cov_matrix(0,0) = gaussian.covariance[0];
        cov_matrix(0,1) = gaussian.covariance[1];
        cov_matrix(1,0) = gaussian.covariance[2];
        cov_matrix(1,1) = gaussian.covariance[3];
    } else if (size == 3)
    {
        mean_pt(0) = gaussian.mean_point.x;
        mean_pt(1) = gaussian.mean_point.y;
        mean_pt(2) = gaussian.mean_point.z;
        cov_matrix(0,0) = gaussian.covariance[0];
        cov_matrix(0,1) = gaussian.covariance[1];
        cov_matrix(0,2) = gaussian.covariance[2];
        cov_matrix(1,0) = gaussian.covariance[3];
        cov_matrix(1,1) = gaussian.covariance[4];
        cov_matrix(1,2) = gaussian.covariance[5];
        cov_matrix(2,0) = gaussian.covariance[6];
        cov_matrix(2,1) = gaussian.covariance[7];
        cov_matrix(2,2) = gaussian.covariance[8];
    } else {
        std::cout << "Error in the GMM definition: Dimention mismatch" << std::endl;
        std::cout << "Mean point dimension: " << mean_pt.size() << std::endl;
        std::cout << "Covariance matrix dimension: " << size << " x " << size << std::endl;
        return -1;
    }

    // std::cout << "Mean point: " << mean_pt.transpose() << std::endl;
    // std::cout << "Covariance matrix: \n" << std::endl << cov_matrix.transpose() << std::endl;    

    float det = cov_matrix.determinant();    
    // std::cout << "Determinant: " << det << std::endl;     

    if (det <= 0)
    {  
        std::cout << "Error! Negative determinant: " << det << std::endl;
        return -1;
    }

    float denom = std::sqrt(std::pow(2*M_PI,size)*det);                                            // denominator
    // std::cout << "Denominator: " << denom << std::endl;

    Eigen::MatrixXd row_matrix;                                                                   // difference between considered pt and mean pt (vector)       
    row_matrix.resize(1,size);
    for (int i=0; i<size; i++)
    {
        row_matrix(0,i) = pt[i] - mean_pt(i);
    }
    Eigen::MatrixXd inv_cov = cov_matrix.inverse();                                         // inverse of covariance matrix 
    Eigen::MatrixXd temp;
    temp.resize(1,1);
    temp = row_matrix * inv_cov * row_matrix.transpose();                                       // pseudo-inverse calculation
    float prob = 1/denom*std::exp(-0.5*temp(0,0));                                              // final value
    // std::cout << "Prob: " << prob << std::endl;

    return prob;
}


float mixture_pdf(turtlebot3_msgs::msg::GMM gmm, std::vector<float> pt)
{
    // std::cout << "Entered multi-component pdf" << std::endl;
    float val = 0;

    // Check if GMM dimensions match
    if (gmm.gaussians.size() != gmm.weights.size())
    {
        std::cout<<"Error in the GMM definition: dimension mismatch"<<std::endl;
        return -1;
    }

    // Check if sum of weights is 1
    float sum_weights = 0;
    for (int i = 0; i < gmm.weights.size(); i++)
    {
        sum_weights += gmm.weights[i];
    }

    std::vector<float> normalized_weights;

    if (sum_weights != 1)
    {
        // DEBUG
        // std::cout<<"Error in the GMM definition: weights do not sum to 1"<<std::endl;
        // std::cout << "Weights vector will be normalized!" << std::endl;
        for (int i = 0; i < gmm.weights.size(); i++)
        {
            normalized_weights.push_back(gmm.weights[i]/sum_weights);
        }
    } else {
        for (int i = 0; i < gmm.weights.size(); i++)
        {
            normalized_weights.push_back(gmm.weights[i]);
        }
    }

    for (int i = 0; i < gmm.gaussians.size(); i++)
    {
        val = val + normalized_weights[i]*single_component_pdf(gmm.gaussians[i], pt);
    }


    return val;
}

//funzione per calcolare il valore di un punto data una serie di distribuzioni a forma di gaussiana 3D
float multiple_gauss3d_pdf(std::vector<std::vector<float>> means_pt, std::vector<std::vector<std::vector<float>>> vars, std::vector<float> pt, std::vector<float> weights){
    float val = 0;

    // devo avere stesso numero di punti medi, varianze e pesi
    if (means_pt.size() != vars.size() || means_pt.size() != weights.size())
    {
        std::cout<<"Error in the GMM definition: dimension mismatch"<<std::endl;
        return -1;
    }

    // calcolo somma dei pesi e controllo che sia uguale a 1
    float sum_weights = 0;
    for (int i = 0; i < weights.size(); i++)
    {
        sum_weights += weights[i];
    }

    if (sum_weights != 1)
    {
        std::cout<<"Error in the GMM definition: weights do not sum to 1"<<std::endl;
        std::cout << "Weights vector will be normalized!" << std::endl;
        for (int i = 0; i < weights.size(); i++)
        {
            weights[i] = weights[i]/sum_weights;
        }
    }

    for (long unsigned int i = 0; i < vars.size(); ++i)
    {
        val = val + weights[i]*gauss3d_pdf(means_pt[i], vars[i], pt);
        // std::cout << "Valore provvisorio: " << val << std::endl;
    }
    return val;
}

//funzione per calcolare il valore di un punto dato un Gaussian Mixture Model
float multiple_gauss3d_pdf2(turtlebot3_msgs::msg::GMM gmm, std::vector<float> pt){
    float val = 0;

    // devo avere stesso numero di punti medi, varianze e pesi
    if (gmm.gaussians.size() != gmm.weights.size())
    {
        std::cout<<"Error in the GMM definition: dimension mismatch"<<std::endl;
        return -1;
    }

    // calcolo somma dei pesi e controllo che sia uguale a 1
    float sum_weights = 0;
    for (int i = 0; i < gmm.weights.size(); i++)
    {
        sum_weights += gmm.weights[i];
    }

    std::vector<float> normalized_weights;

    if (sum_weights != 1)
    {
        std::cout<<"Error in the GMM definition: weights do not sum to 1"<<std::endl;
        std::cout << "Weights vector will be normalized!" << std::endl;
        for (int i = 0; i < gmm.weights.size(); i++)
        {
            normalized_weights.push_back(gmm.weights[i]/sum_weights);
        }
    } else {
        for (int i = 0; i < gmm.weights.size(); i++)
        {
            normalized_weights.push_back(gmm.weights[i]);
        }
    }

    for (int i = 0; i < gmm.gaussians.size(); i++)
    {
        val = val + normalized_weights[i]*single_component_pdf(gmm.gaussians[i], pt);
    }


    return val;
}



//funzione per calcolare il valore di un punto data una serie di distribuzioni a forma di gaussiana
template<typename T>
double multiple_gauss_pdf(std::vector<Vector2<T>> means_pt, std::vector<T> vars, Vector2<T> pt){
    double val = 0;
    for (long unsigned int i = 0; i < vars.size(); ++i)
    {
        val = val + gauss_pdf_std(means_pt[i], vars[i], pt);
    }
    return val;
}

// Prende la somma di tutte le gaussiane standard, le ribalta attorno all'asse x e poi trasla in su di 1 in modo che sia compresa in [0,1] ([k,1] in realta)
template<typename T>
double multiple_contin_gauss_pdf(std::vector<Vector2<T>> means_pt, std::vector<T> vars, Vector2<T> pt, std::vector<T> &k_vect, std::vector<bool> fixed){
    double val = 0;
    double temp = 0;
    for (long unsigned int i = 0; i < vars.size(); ++i)
    {
        temp = temp + gauss_pdf(means_pt[i], vars[i], pt, k_vect[i], fixed[i]);
    }

    val = -temp + 3;             // restituisce valore complessivo ribaltato attorno all'asse x e traslato in su di 1   
    // std::cout << val << std::endl;
    
    // if (val < 0)
    // {
    //     val = 0;
    // }
    // if (val > 3)
    // {
    //     std::cout << val << std::endl;
    // }
    return val; 
}


// Prende la somma di tutte le gaussiane standard, le ribalta attorno all'asse x e poi trasla in su di 1 in modo che sia compresa in [0,1] ([k,1] in realta)
// IGNORA LE GAUSSIANE NASCOSTE DIETRO GLI OSTACOLI
template<typename T>
double multiple_visible_gauss_pdf(std::vector<Vector2<T>> means_pt, std::vector<T> vars, Vector2<T> pt, std::vector<T> &k_vect, std::vector<bool> fixed){
    double val = 0;
    double temp = 0;
    // bool doIntersect = false;
    
    for (long unsigned int i = 0; i < vars.size(); ++i)
    {
        // // Controllo se la gaussiana e nascosta dietro a un ostacolo (guardo se il segmento che congiunge il robot con la gaussiana interseca almeno un lato dell'ostacolo)
        // for (int j=0; j<ObstacleBoxes.size(); j++)
        // {
        //     // Cerco intersezione con singolo ostacolo
        //     doIntersect = doIntersect || intersectLineAndObstacle(pt, means_pt[i], ObstacleBoxes[j]);

        //     // Se il segmento robot-gaussiana non interseca nessun ostacolo tengo conto del peso di quella gaussiana, altrimenti no
        //     if (!doIntersect)
        //     {
        //         temp = temp + gauss_pdf(means_pt[i], vars[i], pt, k_vect[i], fixed[i]);
        //     } 
        //     // else
        //     // {
        //     //     std::cout << "Intersection found: Gauss num " << i << " with obstacle num. " << j << std::endl;
        //     // }
        // }
        // doIntersect = false;                    // reset flag prima di passare alla prossima gaussiana

        // Considero la gaussiana solo se vicina
        // double dist = sqrt(std::pow((pt.x-means_pt[i].x),2) + std::pow((pt.y-means_pt[i].y),2));
        // if (dist < 5)
        // {
        // }
        temp = temp + gauss_pdf(means_pt[i], vars[i], pt, k_vect[i], fixed[i]);

    }

    val = (-temp + vars.size());             // restituisce valore complessivo ribaltato attorno all'asse x e traslato in su di 1   
    // std::cout << val << std::endl;

    if (val < 0)
    {
        val = 0;
    }


    // if (val > 1)
    // {
    //     std::cout << val << std::endl;
    // }
    return val; 

}


//funzione per calcolare il valore di un punto data una distribuzione gaussiana repulsiva troncata a 1
template<typename T>
double repulsive_gauss_pdf(Vector2<T> mean_pt, T var, Vector2<T> pt, T &k, bool fixed){
    double temp = (std::pow((pt.x-mean_pt.x),2.0) + std::pow((pt.y-mean_pt.y),2.0)) / (2.0 * std::pow(var,2.0));
    double temp2 = std::exp(-temp);
    double temp3 = 1/temp2 - 1;                         // Ribalto e abbasso di 1 per avere il minimo in 0                
    double val = 0;
    // if (fixed)                                          // Se fissa taglio a 1
    //     if (temp3 > 1)                                      // Taglio i valori > 1
    //     {
    //         val = 1;
    //     } 
    //     else if (temp3 < k)
    //     {
    //         val = k;
    //     } else
    //     {
    //         val = temp3;
    //     } else {                                          // Se non fissa taglio a 0.5                             
    //         if (temp3 > 0.5)                                      // Taglio i valori > 0.5
    //         {
    //             val = 0.5;
    //         } 
    //         else if (temp3 < k)
    //         {
    //             val = k;
    //         } else
    //         {
    //             val = temp3;
    //         }
    //     }
  
    if (temp3 > 1)                                      // Taglio i valori > 1
    {
        val = 1;
    }
    else if (temp3 < k)
    {
        val = k;
    } else
    {
        val = temp3;
    }

    // if (fixed)                              // se fissa taglio a 1
    // {
    //     if (temp3 > 1)
    //     {
    //         val = 1;
    //     }
    //     else if (temp3 < k)
    //     {
    //         val = k;
    //     }
    //     else
    //     {
    //         val = temp3;
    //     }
        
    // } else                                  // se mobile taglio a 0.5
    // {   
    //     if (temp3 > 0.5)
    //     {
    //         val = 0.5;
    //     }
    //     else if (temp3 < k)
    //     {
    //         val = k;
    //     }
    //     else
    //     {
    //         val = temp3;
    //     }
    // }

    return val;
}

//funzione per calcolare il valore di un punto data una serie di distribuzioni gaussiane repulsive
template<typename T>
double multiple_repul_gauss_pdf(std::vector<Vector2<T>> means_pt, std::vector<T> vars, Vector2<T> pt, std::vector<T> &k_vect, std::vector<bool> fixed){
    double tmp = 0;
    double val = 0;
    double max = 0;
    double fixed_counter = 0;
    for (long unsigned int i = 0; i < vars.size(); ++i)
    {
        tmp = tmp + repulsive_gauss_pdf(means_pt[i], vars[i], pt, k_vect[i], fixed[i]);
        // if (fixed[i])
        // {
        //     fixed_counter = fixed_counter + 1;
        // } 
    }
    tmp = tmp - (vars.size()-1);
    // tmp = tmp - 0.5*(vars.size() -1) -0.5*(fixed_counter-1);         // -0.5 per ogni gaussiana, poi un altro -0.5 per ogni fissa
    if (tmp < 0)
    {
        val = 0;
    } else
    {
        val = tmp;
    }
    return val;
}

//***********************************************************************************************************************************

//***********************************************************************************************************************************
//------------------------------------ Update Robots positions - Simulate Robots motion to centroids --------------------------------

//Se robot conosce la sua posizione globale iniziale magari la funzione updatepoints potr� ragionare su mGlobalPoint (ed eventualmente su un mCentroid)
//tutte le seguenti funzioni in pratica andranno spostate in Diagram.h poich� diventeranno parte integrante del singolo diagramma
//Aggiornamento vettore globale "points" (da utilizzare dopo calcolo vettore centroidi per poter ricalcolare i diagrammi con le nuove posizioni)
template<typename T>
std::vector<Vector2<T>> updatepoints(const std::vector<Diagram<T>>& diagrams, const std::vector<Vector2<T>>& centroids)
{
    auto updt_points = std::vector<Vector2<T>>();
    auto global_points = std::vector<Vector2<T>>();

    //Raccogliamo gli mGlobalPoint correnti in un vettore
    for(const auto& diagram : diagrams){
        global_points.push_back(diagram.getGlobalPoint());
    }
    //Aggiungiamo al valore points[i] originario il valore del centroide[i](ovvero lo spostamento locale rispetto a {0.0,0.0})
    for(std::size_t i=0; i<global_points.size(); ++i){
        auto updt_point = global_points[i] + centroids[i];
        updt_points.push_back(updt_point);
    }

    return updt_points;
}

//Update posizione globale
template<typename T>
Vector2<T> updateGlobalPoint(const Diagram<T>& diagram, const Vector2<T>& centroid)
{
    //Aggiungiamo al valore point originario il valore del centroide(ovvero lo spostamento locale rispetto a {0.0,0.0})
    auto updt_point = diagram.getGlobalPoint() + centroid;

    return updt_point;
}
//***********************************************************************************************************************************

#endif // VORONOI_H_INCLUDED
