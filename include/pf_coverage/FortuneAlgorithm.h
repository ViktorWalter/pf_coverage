#pragma once
// STL
#include <unordered_map>
// My includes
#include "PriorityQueue.h"
#include "Diagram.h"
#include "Beachline.h"
#include "Event.h"
#include "Utility.h"
//Le sezioni //// sono sezioni di debug a terminale messe per la stampa dettagliata delle strutture dati a terminale

/*  Esecuzione dell'algoritmo Fortune per la generazione del Diagramma di Voronoi.
    L'algoritmo Fortune è diretto nel senso che non necessita della costruzione del
    grafo di Delaunay. La triangolazione di Delaunay può eventualmente essere calcolata
    in seguito.

    Pseudo-codice dell'algoritmo Fortune:
     add a SiteEvent to the PriorityQueue for each input Site
     while PriorityQueue is not empty {
        pop (rimuovi) the top event;
        if the popped event is a SiteEvent {
            insert a new arc triplet(leftArc, middleArc(replace aboveArc), rightArc) in the Beachline;
            check for new CircleEvents;
        }
        else (popped event is a CircleEvent) {
            create a Vertex in the diagram;
            remove the shrinkedArc from the Beachline;
            delete invalidated events;
            check for new CircleEvents;
        }
     }

    -Algoritmo Fortune salta da un evento all'altro in ordine di coordinata y decrescente.
    -Algoritmo Fortune restituisce di base un diagramma illimitato quindi è necessario un
     algoritmo di bounding per limitare il diagramma, clippando gli edge infiniti.
    -Algoritmo di Bounding sulla carta è sufficientemente indipendente dall'algoritmo di
     generazione del diagramma di Voronoi (quindi può essere adattato alle esigenze reali di percezione limitata?)
    -E' possibile effettuare l'intersezione del diagramma di Voronoi con una Box (teoricamente di qualsiasi forma,
     qui rettangolare) --> Algoritmo di intersezione.
*/

template<typename T>
class FortuneAlgorithm
{
public:
    //Costruttore dell'algoritmo a partire dal set di punti input
    explicit FortuneAlgorithm(std::vector<Vector2<T>> points, Box<T> box) : mDiagram(std::move(points), std::move(box))
    {}
    //Costruttore dell'algoritmo a partire dal set di punti input
    explicit FortuneAlgorithm(std::vector<Vector2<T>> points, Box<T> box, Vector2<T> point_global) : mDiagram(std::move(points), std::move(box), std::move(point_global))
    {}

    //Metodi Principali
    //Costruzione del Diagramma di Voronoi
    void construct()
    {
        //Inizializzazione della PriorityQueue con tutti i SiteEvents
        for (auto i = std::size_t(0); i < mDiagram.getNbSites(); ++i)
            //Inserimento dei SiteEvents nella queue mEvents
            mEvents.push(std::make_unique<Event<T>>(mDiagram.getSite(i)));  //costruisce evento associandolo al Site

//        ////Lista ordinata degli eventi processati
//        std::cout << "Lista ordinata degli eventi processati:" << std::endl;

        //Elaborazione eventi della PriorityQueue fino alla svuotamento della stessa
        while (!mEvents.isEmpty())
        {
//            ////Stampa il nodo in posizione top della queue --> considerando che è in un ciclo while, viene stampata la lista degli eventi in ordine di elaborazione (debug)
//            mEvents.print(std::cout);

            auto event = mEvents.pop(); //rimozione del top event della queue, quello con coordinata y maggiore
            mSweepLineY = event->y;     //salvo la coordinata y dell'evento (Sweepline)
            //E' un SiteEvent
            if (event->type == Event<T>::Type::Site){
                handleSiteEvent(event.get());   //funzione di gestione di un SiteEvent
//                ////Stampa RB-Tree (debug)
//                mBeachline.print(std::cout);
            }
            //E' un CircleEvent
            else{
                handleCircleEvent(event.get()); //funzione di gestione di un CircleEvent
//                ////Stampa RB-Tree (debug)
//                mBeachline.print(std::cout);
            }
        }
//        ////Beachline completa
//        std::cout << "\nRB-Tree/Beachline Completa" << std::endl;
//        std::cout << "Root: ";
//        mBeachline.print(std::cout);
    }

    //Algoritmo di Bounding del Diagramma di Voronoi (richiede di aver creato prima il diagramma e prende in input una BoundingBox)
    bool bound(Box<T> box)
    {
        auto success = true;
        //1. Assicurarsi che la BoundingBox fornita contenga tutti i vertici del diagramma di Voronoi al proprio interno
        for (const auto& vertex : mDiagram.getVertices()) //ciclo for ripetuto per tutti i vertici del diagramma
        {   //Eventualmente in caso non li contenga tutti si provvede alla estensione della box per comprendere tutti i vertici del diagramma
            box.left = std::min(vertex.point.x, box.left);
            box.bottom = std::min(vertex.point.y, box.bottom);
            box.right = std::max(vertex.point.x, box.right);
            box.top = std::max(vertex.point.y, box.top);
        }
        //2. Clippare gli edge infiniti (fondamentale) --> step abbastanza semplice poiché si procede al calcolo delle intersezioni tra i lati del diagramma e i lati della BoundingBox
        // Si definiscono due variabili: un vector per immagazzinare tutti i punti di intersezione
        auto linkedVertices = std::list<LinkedVertex>(); //lista delle struct LinkedVertex
        auto vertices = VerticeOnFrontierContainer(mDiagram.getNbSites());  //vertici sulla frontiera originati dalla fase di clippaggio del diagramma con la bounding box
        //Al termine dell'algoritmo gli half-edge infiniti sono quelli puntati dai nodi/archi rimanenti all'interno della Beachline
        if (!mBeachline.isEmpty())  //Se la beachline non è vuota (e non lo è)
        {   //Quindi a partire dal leftmost arc della beachline fino al rightmost, attraverso un ciclo while si procede al clippaggio di tutti gli edge infiniti residui con la BoundingBox (funzione boundedge)
            //Operazione ripetuta per tutti gli half-edge puntati dagli archi della beachline, scansionati uno per volta
            auto arc = mBeachline.getLeftmostArc(); //partiamo dal leftmostArc
            while (!mBeachline.isNil(arc->next))    //finché non sono stati scansionati tutti gli archi della beachline
            {   //Clippaggio diagramma
                success = boundEdge(box, arc, arc->next, linkedVertices, vertices) && success;
                arc = arc->next;    //passaggio all'arco successivo della beachline
            }
        }
        //3. Chiudere le celle --> step che richiede più accortezza, suddivisibile in due sottostep principali
        //3.1 Aggiungere corners alla box per le celle che ne necessitano
        for (auto& kv : vertices)       //per tutte le struct LinkedVertex della frontiera
            success = addCorners(box, linkedVertices, kv.second) && success;    //aggiungiamo i corner della box
        //3.2. Unire gli half-edge relativi ai vertici sulla frontiera in modo che ogni cella sulla frontiera del diagramma venga opportunamente chiusa
        for (auto& kv : vertices)       //per tutte le struct LinkedVertex della frontiera
            joinHalfEdges(kv.first, kv.second);     //concatenazione dei vertici sulla frontiera/box
        //Return the status
        return success;
    }

    //Metodo Getter del Diagramma di Voronoi
    Diagram<T> getDiagram()
    {
        return std::move(mDiagram);
    }

private:
    //Variabili interne
    Diagram<T> mDiagram;                //Diagramma di Voronoi DCEL
    Beachline<T> mBeachline;            //Beachline RB-Tree
    PriorityQueue<Event<T>> mEvents;    //PriorityQueue degli eventi
    T mSweepLineY;                      //Coordinata y corrente della Sweepline

    //Gestione SiteEvent
    void handleSiteEvent(Event<T>* event)
    {
        auto site = event->site;    //salvo il sito associato al SiteEvent da gestire
        //1. Controlliamo innanzitutto se si tratta del primo sito del diagramma
        if (mBeachline.isEmpty())
        {   //Creazione nuovo arco associato a quel sito + Set nuovo arco come Root
            mBeachline.setRoot(mBeachline.createArc(site));
            return;
        }
        //2. Individuaiamo l'arco esistente su cui si innesta il nuovo arco associato al sito in esame
        auto arcToBreak = mBeachline.locateArcAbove(site->point, mSweepLineY);
        deleteEvent(arcToBreak); //Eliminiamo l'evento associato all'arco individuato
        //3. Sostituiamo l'arco individuato con una tripletta di archi (middleArc, leftArc, rightArc)
        auto middleArc = breakArc(arcToBreak, site); //replace del vecchio arco con il nuovo middleArc
        auto leftArc = middleArc->prev;  //agganciamo il nuovo middleArc al leftArc
        auto rightArc = middleArc->next; //agganciamo il nuovo middleArc al rightArc
        //4. Aggiungiamo il growing edge definito dal nuovo middleArc all'interno del diagramma DCEL
        addEdge(leftArc, middleArc);    //creazione growing half-edges + associazione puntatori hlaf-edges del leftArc e del middleArc
        middleArc->rightHalfEdge = middleArc->leftHalfEdge;
        rightArc->leftHalfEdge = leftArc->rightHalfEdge;  //associazione puntatori half-edges del middleArc e del rightArc
        //5. Check di eventuali CircleEvents di intersezione tra edge di due middleArcs diversi (circle events da aggiunegere eventualmente alla queue)
        //Left triplet (checkiamo se il precedente del leftArc è una foglia: se non lo è vuol dire che anche lui ha un growing edge quindi si deve calcolare il breakpoint che prima o poi si formerà a sinista del middleArc)
        if (!mBeachline.isNil(leftArc->prev))
            addEvent(leftArc->prev, leftArc, middleArc); //aggiunta del CircleEvent alla queue
        //Right triplet (checkiamo se il successivo del rightArc è una foglia: se non lo è vuol dire che anche lui ha un growing edge quindi si deve calcolare il breakpoint che prima o poi si verificherà a destra del middleArc)
        if (!mBeachline.isNil(rightArc->next))
            addEvent(middleArc, rightArc, rightArc->next);  //aggiunta del CircleEvent alla queue
    }
    //Gestione CircleEvent
    void handleCircleEvent(Event<T>* event)
    {
        auto point = event->point;  //salvo il vertice/convergence point che si creerà con l'evento
        auto arc = event->arc;      //salvo l'arco associato all'evento che verrà shrinkatto con lo stesso
        //1. Aggiungiamo il vertice di Voronoi al diagramma di Voronoi DCEL
        auto vertex = mDiagram.createVertex(point);
        //2. Eliminiamo tutti gli eventi associati all'arco "arc" da eliminare
        auto leftArc = arc->prev;   //set del puntatore prev
        auto rightArc = arc->next;  //set del puntatore next
        deleteEvent(leftArc);   //eliminazione del circle event associato all'arco precedente di "arc"
        deleteEvent(rightArc);  //eliminazione del circle event associato all'arco successivo di "arc"
        //3. Update della beachline (rimozione "arc") e del diagramma DCEL
        removeArc(arc, vertex);
        //4. Check di eventuali nuovi CircleEvents/Eventi di intersezione
        //Left triplet
        if (!mBeachline.isNil(leftArc->prev))
            addEvent(leftArc->prev, leftArc, rightArc);
        //Right triplet
        if (!mBeachline.isNil(rightArc->next))
            addEvent(leftArc, rightArc, rightArc->next);
    }

    //Replace dell'arco originario above "arc" con la tripletta di nuovi archi
    Arc<T>* breakArc(Arc<T>* arc, typename Diagram<T>::Site* site)
    {
        //Creazione del nuovo middleArc associato al site del siteEvent
        auto middleArc = mBeachline.createArc(site);
        //Split del vecchio "arc" in due sottoarchi (leftArc, rightArc)
        auto leftArc = mBeachline.createArc(arc->site, Arc<T>::Side::Left);
        leftArc->leftHalfEdge = arc->leftHalfEdge;  //associazione leftHalfEdge
        auto rightArc = mBeachline.createArc(arc->site, Arc<T>::Side::Right);
        rightArc->rightHalfEdge = arc->rightHalfEdge;   //associazione rightHalfEdge
        //Update Beachline
        //Replace del vecchio "arc" con il nuovo middleArc (middleArc eredita come figli nel tree leftArc e rightArc)
        mBeachline.replace(arc, middleArc);
        //Inserimento del leftArc prima del middleArc
        mBeachline.insertBefore(middleArc, leftArc);
        //Inserimento del rightArc dopo il middleArc
        mBeachline.insertAfter(middleArc, rightArc);

        //Cancellazione del vecchio "arc"
        delete arc;
        //Restituzione puntatore al middleArc
        return middleArc;
    }
    //Rimozione dell'arco "shrinked into nothing" e pertanto non più necessario
    void removeArc(Arc<T>* arc, typename Diagram<T>::Vertex* vertex)
    {
        //Terminiamo i due edges che si sono intersecati, originando il vertice di Voronoi
        setDestination(arc->prev, arc, vertex); //vedi quaderno
        setDestination(arc, arc->next, vertex); //vedi quaderno
        //Uniamo gli HalfEdge sx e dx dell'arc (arc che ora coincide geometricamente con il vertice di Voronoi)
        arc->leftHalfEdge->next = arc->rightHalfEdge;
        arc->rightHalfEdge->prev = arc->leftHalfEdge;
        //Rimozione dell'arco "arc" che è "shrinked into nothing" e quindi non serve più
        mBeachline.remove(arc);
        //Creazione del nuovo edge (tracciato a partire dal vertice di Voronoi)
        auto prevHalfEdge = arc->prev->rightHalfEdge;   //lo salvo per dopo
        auto nextHalfEdge = arc->next->leftHalfEdge;    //lo salvo per dopo
        //Inserimento del nuovo edge
        addEdge(arc->prev, arc->next);
        setOrigin(arc->prev, arc->next, vertex);    //set dell'origine degli half-edge
        //Concatenazione half-edges nuovi con quelli già esistenti
        setPrevHalfEdge(arc->prev->rightHalfEdge, prevHalfEdge);    //logica intuitiva vedi immagine c quaderno
        setPrevHalfEdge(nextHalfEdge, arc->next->leftHalfEdge);     //logica intuitiva vedi immagine c quaderno
        //Eliminazione nodo/arco
        delete arc;
    }

    //Capire se il breakpoint traccia un lato verso sx o verso dx man mano che la sweepline avanza
    bool isMovingRight(const Arc<T>* left, const Arc<T>* right) const
    {   //se la coordinata y del sito dell'arco left è minore della y del sito dell'arco right, il breakpoint si muove verso destra
        return left->site->point.y < right->site->point.y;
    }
    //Fornisce la coordinata x del sito dell'arco left se il breakpoint si sta muovendo verso destra oppure quella del sito dell'arco right se si sta muovendo a sinistra
    T getInitialX(const Arc<T>* left, const Arc<T>* right, bool movingRight) const
    {
        return movingRight ? left->site->point.x : right->site->point.x;
    }

    //Aggiunta di un nuovo edge dati gli archi sx e dx
    void addEdge(Arc<T>* left, Arc<T>* right)
    {
        //Creazione dei due half-edges associati
        left->rightHalfEdge = mDiagram.createHalfEdge(left->site->face);    //nuovo right half-edge per l'arco a sx
        right->leftHalfEdge = mDiagram.createHalfEdge(right->site->face);   //nuovo left half-edge per l'arco a dx
        //Si settano i due half-edges come twin/gemelli
        left->rightHalfEdge->twin = right->leftHalfEdge;
        right->leftHalfEdge->twin = left->rightHalfEdge;
    }
    //Settiamo il vertice di origine di un edge
    void setOrigin(Arc<T>* left, Arc<T>* right, typename Diagram<T>::Vertex* vertex)
    {
        left->rightHalfEdge->destination = vertex;  //half-edge rappresentato a destra dell'edge, ordine anti-orario (vedi quaderno per capire)
        right->leftHalfEdge->origin = vertex;
    }
    //Settiamo il vertice di destinazione di un edge
    void setDestination(Arc<T>* left, Arc<T>* right, typename Diagram<T>::Vertex* vertex)
{
    left->rightHalfEdge->origin = vertex;       //half-edge rappresentato a destra dell'edge, ordine anti-orario (vedi quaderno per capire)
    right->leftHalfEdge->destination = vertex;
}
    //Concatenazione nuovo half-edge con quelli esistenti delle due facce adiacenti
    void setPrevHalfEdge(typename Diagram<T>::HalfEdge* prev, typename Diagram<T>::HalfEdge* next)
    {
        prev->next = next;
        next->prev = prev;
    }

    //Aggiunta di un futuro CircleEvent alla PriorityQueue (feedforward)
    void addEvent(Arc<T>* left, Arc<T>* middle, Arc<T>* right)
    {
        auto y = T();   //coordinata y della sweepline a cui si verificherà il CircleEvent
        //Calcolo del breakpoint/punto di intersezione tra il growing edge dell'arco left e il growing edge dell'arco right
        auto convergencePoint = computeConvergencePoint(left->site->point, middle->site->point, right->site->point, y);
        //Controlliamo che la coordinata y a cui si verificherà sia minore dell'attuale posizione y della sweepline, se non lo è l'evento non è valido
        if (!almostLower(y, mSweepLineY))
            return;
        //Controlliamo che l'evento dia valido, checkando se il middleArc verrà shrinkato/distrutto dalla convergeneza dei due breakpoints
        //per farlo controlliamo in che direzione si stanno muovendo i breakpoint degli archi left e right
        auto leftBreakpointMovingRight = isMovingRight(left, middle);
        auto rightBreakpointMovingRight = isMovingRight(middle, right);
        auto leftInitialX = getInitialX(left, middle, leftBreakpointMovingRight);
        auto rightInitialX = getInitialX(middle, right, rightBreakpointMovingRight);
        if (!(!leftBreakpointMovingRight && rightBreakpointMovingRight) &&
            ((leftBreakpointMovingRight && almostLower(leftInitialX, convergencePoint.x)) ||
            (!leftBreakpointMovingRight && almostGreater(leftInitialX, convergencePoint.x))) &&
            ((rightBreakpointMovingRight && almostLower(rightInitialX, convergencePoint.x)) ||
            (!rightBreakpointMovingRight && almostGreater(rightInitialX, convergencePoint.x))))
        {
            //Creiamo il CircleEvent e aggiungiamolo alla PriorityQueue
            auto event = std::make_unique<Event<T>>(y, convergencePoint, middle);   //middle è l'arco che verrà shrinkato dall'evento
            middle->event = event.get();        //associamo all'arco middle l'evento appena creato
            mEvents.push(std::move(event));     //inseriamo il CircleEvent creato nella PriorityQueue ordinato secondo il suo valore y
        }
    }
    //Eliminazione di un evento --> si avvale del metodo remove della PriorityQueue
    void deleteEvent(Arc<T>* arc)
    {
        if (arc->event != nullptr)  //se l'arco punta ad un evento della queue
        {
            mEvents.remove(arc->event->index);  //rimuoviamo l'evento dalla queue
            arc->event = nullptr;               //azzeriamo il puntatore a nullptr
        }
    }
    //Calcolo del vertice di Voronoi dati tre siti (point1,point2,point3) e la coordinata della sweepline
    Vector2<T> computeConvergencePoint(const Vector2<T>& point1, const Vector2<T>& point2, const Vector2<T>& point3, T& y) const
    {
        auto v1 = (point1 - point2).getOrthogonal();    //bisettore delle celle di point1 e point2
        auto v2 = (point2 - point3).getOrthogonal();    //bisettore delle celle di point2 e point3
        auto delta = static_cast<T>(0.5) * (point3 - point1);   //punto intermedio
        auto denom = v1.getDet(v2);
        //Punti allineati, quindi non c'è soluzione
        if (almostZero(denom))
        {   //Infinity means that the event will never be added to the event queue
            y = std::numeric_limits<T>::infinity();
            return Vector2<T>();
        }
        //Punti non allineati --> esiste un convergence point
        auto t = delta.getDet(v2) / denom;
        auto center = static_cast<T>(0.5) * (point1 + point2) + t * v1; //centro della circonferenza --> futuro vertice di Voronoi
        auto r = center.getDistance(point1);    //raggio della circonferenza
        y = center.y - r;   //coordinata y della sweepline alla quale si verificherà il CircleEvent esaminato
        return center;
    }

    //Bounding --> struct per il bounding del diagramma
    struct LinkedVertex
    {
        typename Diagram<T>::HalfEdge* prevHalfEdge;
        typename Diagram<T>::Vertex* vertex;
        typename Diagram<T>::HalfEdge* nextHalfEdge;
    };

    //Frontiera costituita da array puntatori (cellVertices) ai vertici delle celle sulla frontiera: ogni cella ha due vertici: vertice sx --> cellVertices[2*side] e vertice dx --> cellVertices[2*side + 1]
    using VerticeOnFrontierContainer = std::unordered_map<std::size_t, std::array<LinkedVertex*, 8>>;

    //Funzione di clippaggio dei lati finali --> calcolo intersezione lati con box
    bool boundEdge(const Box<T>& box, Arc<T>* leftArc, Arc<T>* rightArc, std::list<LinkedVertex>& linkedVertices,
        VerticeOnFrontierContainer& vertices)
    {
        auto success = true;
        //Clippaggio di un lato
        auto direction = (leftArc->site->point - rightArc->site->point).getOrthogonal();        //direzione del bisettore dei due siti
        auto origin = (leftArc->site->point + rightArc->site->point) * static_cast<T>(0.5);     //punto intermedio tra i due siti
        //Edge-box intersection
        auto intersection = box.getFirstIntersection(origin, direction);    //calcolo intersezione
        //Creazione di un nuovo vertice e terminazione degli half-edges
        auto vertex = mDiagram.createVertex(intersection.point);    //creiamo il vertice nel punto di intersezione
        setDestination(leftArc, rightArc, vertex);  //terminiamo half-edges in vertex
        //Inizializzazione puntatori ai vertici (index è l'ID del sito dell'arco)
        if (vertices.find(leftArc->site->index) == vertices.end())  //cerchiamo gli array nell'unordered_map attraverso la key (.first), e come key prendiamo l'indice del sito del leftArc
            //Dualità (unordered_)map-array, quindi si può sfruttare la notazione associative degli array: elem[key]
            vertices[leftArc->site->index].fill(nullptr);   //ovviamente se non è stata settato in precedenza non verrà trovato, quindi inizializziamolo
            //così abbiamo inizializzato a nullptr tutto l'array di puntatori corrispondenti alla key index interno al map
        if (vertices.find(rightArc->site->index) == vertices.end())
            vertices[rightArc->site->index].fill(nullptr);
        //Checkiamo che i vertici non siano già stati assegnati
        success = vertices[leftArc->site->index][2 * static_cast<int>(intersection.side) + 1] == nullptr && success;    //vertice dx dell'arco
        success = vertices[rightArc->site->index][2 * static_cast<int>(intersection.side)] == nullptr && success;       //vertice sx dell'arco
        //Imagazziniamo i vertici sul contorno/frontiera nella list LinkedVertices
        linkedVertices.emplace_back(LinkedVertex{nullptr, vertex, leftArc->rightHalfEdge}); //vertice dx cella (ha un half-edge solo a dx-->vedi quaderno)
        vertices[leftArc->site->index][2 * static_cast<int>(intersection.side) + 1] = &linkedVertices.back();
        linkedVertices.emplace_back(LinkedVertex{rightArc->leftHalfEdge, vertex, nullptr}); //vertice sx cella (ha un half-edge solo a sx-->vedi quaderno)
        vertices[rightArc->site->index][2 * static_cast<int>(intersection.side)] = &linkedVertices.back();
        //Return the status
        return success;
    }
    //Aggiungiamo i corners/angoli alla bounding box
    bool addCorners(const Box<T>& box, std::list<LinkedVertex>& linkedVertices, std::array<LinkedVertex*, 8>& cellVertices)
    {
        auto success = true;
        //Controlliamo due volte il primo side della box (all'inizio e alla fine) per essere sicuri di aggiungere tutti i corners
        for (auto i = std::size_t(0); i < 5; ++i)
        {
            auto side = i % 4;  //4 side della box (0,1,2,3)
            auto nextSide = (side + 1) % 4; //4 nextSide della box (1,2,3,0)
            //Aggiunta del primo(sx) corner
            if (cellVertices[2 * side] == nullptr && cellVertices[2 * side + 1] != nullptr) //in altri termini se siamo nel LinkedVertex iniziale/sx di un side della box
            {
                auto prevSide = (side + 3) % 4; //4 prevSide della box (3,0,1,2)
                //Creazione del vertex corner
                auto corner = mDiagram.createCorner(box, static_cast<typename Box<T>::Side>(side));
                //Aggiungiamo il vertex corner alla list dei LinkedVertex. Il corner non ha half-edge
                linkedVertices.emplace_back(LinkedVertex{nullptr, corner, nullptr});
                //Check che non stiamo per caso cancellando un vertice esistente proprio nel corner dell'ultima cella side precedente
                success = cellVertices[2 * prevSide + 1] == nullptr && success; //se la cella precedente non ha un vertice proprio nel corner allora il suo vertice destro è nullptr poiché ultima cella del Side
                //Immagazziniamo il vertice corner nella frontiera/map
                cellVertices[2 * prevSide + 1] = &linkedVertices.back();
                cellVertices[2 * side] = &linkedVertices.back();
            }
            //Aggiunta del secondo(dx) corner
            else if (cellVertices[2 * side] != nullptr && cellVertices[2 * side + 1] == nullptr) //in altri termini se siamo nel LinkedVertex finale/dx di un side della box
            {   //Creazione del vertex corner
                auto corner = mDiagram.createCorner(box, static_cast<typename Box<T>::Side>(nextSide));
                //Aggiungiamo il vertex corner alla list dei LinkedVertex. Il corner non ha half-edge
                linkedVertices.emplace_back(LinkedVertex{nullptr, corner, nullptr});
                //Check se non stiamo cancellando un vertice esistente proprio nel corner della prima cella side successivo
                success = cellVertices[2 * nextSide] == nullptr && success;
                //Imagazziniamo il vertice corner nella frontiera/map
                cellVertices[2 * side + 1] = &linkedVertices.back();
                cellVertices[2 * nextSide] = &linkedVertices.back();
            }
        }
        // Return the status
        return success;
    }
    //Metodo per concatenare i vertici sulla frontiera unendone gli half-edge
    void joinHalfEdges(std::size_t i, std::array<LinkedVertex*, 8>& cellVertices)
    {   //Analizziamo tutti i side della box (partendo da quello left)
        for (auto side = std::size_t(0); side < 4; ++side)
        {
            if (cellVertices[2 * side] != nullptr)  //vertici di sx sulla frontiera non ancora linkati tra loro
            {   //Linkiamo i vertici tramite half-edges
                auto halfEdge = mDiagram.createHalfEdge(mDiagram.getFace(i));   //i è la key del map ovvero l'ID del sito associato alla faccia
                halfEdge->origin = cellVertices[2 * side]->vertex;  //vertice di origine dell'half-edge: vertice sx cella
                halfEdge->destination = cellVertices[2 * side + 1]->vertex; //vertice di destinazione dell'half-edge: vertice dx cella
                //Concatenamento degli half-edges
                cellVertices[2 * side]->nextHalfEdge = halfEdge;
                halfEdge->prev = cellVertices[2 * side]->prevHalfEdge;
                if (cellVertices[2 * side]->prevHalfEdge != nullptr)    //se non si tratta del vertice corner sx di un side della box, agganciamolo come next
                    cellVertices[2 * side]->prevHalfEdge->next = halfEdge;
                cellVertices[2 * side + 1]->prevHalfEdge = halfEdge;
                halfEdge->next = cellVertices[2 * side + 1]->nextHalfEdge;
                if (cellVertices[2 * side + 1]->nextHalfEdge != nullptr)    //se non si tratta del vertice corner dx di un side della box, agganciamolo come prev
                    cellVertices[2 * side + 1]->nextHalfEdge->prev = halfEdge;
            }
        }
    }
};
