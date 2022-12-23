#pragma once
// My includes
#include "Vettore.h"
#include "Diagram.h"

//Prototipo classe (necessario perché richiamata nella classe Event)
template<typename T>
class Arc;

//Classe Event --> oggetti/elementi del container di eventi PriorityQueue
template<typename T>
class Event
{
public:
    //Distinzione tra due tipi di evento
    enum class Type{Site, Circle};
    //Costruttore Site Event (valore index evento di default = -1, tanto poi lo si setta in fase di inserimento nella Queue con il metodo push)
    explicit Event(typename Diagram<T>::Site* site) : type(Type::Site), x(site->point.x), y(site->point.y), index(-1), site(site)   //Associazione del sito all'evento
    {}
    //Costruttore Circle event (valore index evento di default = -1, tanto poi lo si setta in fase di inserimento nella Queue con il metodo push)
    Event(T y, Vector2<T> point, Arc<T>* arc) : type(Type::Circle), x(point.x), y(y), index(-1), point(point), arc(arc)
    {}

    //Variabili interne pubbliche (così sono accessibili dall'esterno)
    const Type type;    //variabile enum identificatrice del tipo di evento
    T x;                //coordinata x del sito associato ad un SiteEvent o coordinata x del p.to di convergenza/vertice originato dal CircleEvent
    T y;                //coordinata y del sito associato ad un SiteEvent o coordinata y della Sweepline in corrispondenza della quale si verifica un CircleEvent
    int index;          //ID dell'evento (stabilito in base alla dimensione della PriorityQueue) in fase di inserimento (vedi metodo push() di PriorityQueue.h)
    //Variabili per Site event
    typename Diagram<T>::Site* site;    //puntatore al sito/generatore del site event
    //Variabili per Circle event
    Vector2<T> point;                   //vertice/p.to di intersezione/convergenza archi creato con il circle event
    Arc<T>* arc;                        //puntatore all'arco di parabola che viene rimosso con il circle event

};

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//FUNZIONE CHIAVE PER LA PRIORITY QUEUE: utilizzata nei metodi siftUp e siftDown per confrontare/risistemare i nodi/eventi della queue in base alla coordinata y dell'evento stesso.
template<typename T>
bool operator<(const Event<T>& lhs, const Event<T>& rhs)
{   //Gli eventi vengono priorizzati in base alla coordinata y oppure, in caso si abbia la stessa y, in base alla coordinata x
    return lhs.y < rhs.y || (lhs.y == rhs.y && lhs.x < rhs.x);  //è qui la chiave di tutto (ordine decrescente y)
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------

//Overload dell'operatore di stream/stampa del singolo evento(connesso alle funzioni di stampa in PriorityQueue.h)
template<typename T>
std::ostream& operator<<(std::ostream& os, const Event<T>& event)
{
    if (event.type == Event<T>::Type::Site) //Stampa SiteEvent (index evento, (coords x,y sito associato))
        os << "S(" << event.site->index << ", " << event.site->point << ")";
    else                                    //Stampa CircleEvent (arco associato eliminato, coord y della sweepline in cui si verifica, (coords x,y vertice creato))
        os << "C(" << event.arc << ", " << event.y << ", " << event.point << ")";
    return os;
}
