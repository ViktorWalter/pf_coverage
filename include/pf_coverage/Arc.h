#pragma once
// My includes
#include "Diagram.h"

//Prototipo classe (necessario perché richiamata nella classe Arc)
template<typename T>
class Event;

//Classe Arc --> arco di parabola della Beachline, ogni oggetto Arc è un nodo dell'RB-Tree con cui la Beachline è rappresentata
template<typename T>
struct Arc
{
    //Colore del nodo (per l'operazione di balncing dell'albero)
    enum class Color{Red, Black};
    //Side dell'arco (rispetto ad un'ipotetica linea verticale immaginaria)
    enum class Side{Left, Right};

    //Gerarchia archi: puntatori agli altri nodi/archi dell'RB-Tree
    Arc<T>* parent;     //puntatore al nodo padre
    Arc<T>* left;       //puntatore al nodo leftchild
    Arc<T>* right;      //puntatore al nodo rightchild

    //Associazioni al Diagramma: puntatori al sito associato e agli half-edges rispettivamente a sx e a dx dell'arco
    typename Diagram<T>::Site* site;                //puntatore al sito associato all'arco
    typename Diagram<T>::HalfEdge* leftHalfEdge;    //puntatore all'half-edge a sx dell'arco
    typename Diagram<T>::HalfEdge* rightHalfEdge;   //puntatore all'half-edge a dx dell'arco
    Event<T>* event;                                //puntatore all'evento associato all'arco

    //Ottimizzazione DCEL: puntatori ad arco successivo e ad arco precedente della beachline (come se la Beachline fosse una DCEL)
    Arc<T>* prev;               //puntatore all'arco precedente della Beachline
    Arc<T>* next;               //puntatore all'arco successivo della Beachline

    //Bilanciamento RB-Tree per miglioramento di efficienza nelle operazioni di search, insert e delete
    //Albero è bilanciato quando, considerato un qualunque nodo, il numero di nodi nel sottoalbero sx differisce al massimo di 1 dal numero dei nodi nel sottoalbero di dx (in questo modo una qualunque opearzione sull'albero impiega un tempo uniforme)
    Color color;    //Bilanciamento dell'albero avviene sfruttando i colori

    //Per capire se l'arco va a -oo o a +oo (utile per calcolo p.ti di intersezione tra archi adiacenti sulla Beachline)
    Side side;
};
