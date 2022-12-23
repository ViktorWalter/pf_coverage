#pragma once
// STL
#include <vector>
#include <list>
#include <unordered_set>
// My includes
#include "Box.h"
#include "Triangulation.h"

/*  Data structure diagramma di Voronoi --> DCEL: Doubly Connected Edge List
    La DCEL prevede 4 struct interne (Site struct, Vertex struct, HalfEdge struct, Face struct)
    associate ai relativi containers (variabili interne del diagramma).
    Concettualmente ogni edge comune a due celle adiacenti è splittato in 2 half-edge, uno di
    proprietà di una cella , l'altro di proprietà della cella adiacente che condivide
    con la prima quell'edge.
    Ogni half-edge ha un vertice di origine e un vertice di destinazione e quindi una
    direzione (convenzionalmente antioraria). Complessivamente, battezzato un primo half-edge
    della cella, procedendo in senso antiorario l'ultimo half-edge di una cella chiude il
    poligono, nel senso che il suo vertice di destinazione si ricongiunge al vertice di origine
    del primo half-edge della cella.
*/
const int DEBUG = 0;

//Prototipo classe (necessario perché richiamata nella classe Diagram)
template<typename T>
class FortuneAlgorithm;

//Classe Diagram --> DCEL Data Structure
template<typename T>
class Diagram
{
public:
    struct HalfEdge;
    struct Face;

    //Struttura di un Site
    struct Site
    {
        std::size_t index;  //Indice ID del sito nel vettore mSites
        Vector2<T> point;   //Coordinate x,y del sito
        Face* face;         //Puntatore alla faccia del sito
    };

    //Struttura di un Vertex
    struct Vertex
    {
        Vector2<T> point;   //Coordinate x,y del vertice

    private:
        friend Diagram<T>;
        typename std::list<Vertex>::iterator it; //iteratore che punta al vertice nella lista mVertices di tutti i vertici del diagramma
    };
    //Struttura di un HalfEdge
    struct HalfEdge
    {
        Vertex* origin = nullptr;       //Puntatore al vertice di origine
        Vertex* destination = nullptr;  //Puntatore al vertice di destinazione
        HalfEdge* twin = nullptr;       //Puntatore al vertice gemello della faccia adiacente
        Face* incidentFace;             //Puntatore alla faccia a cui l'half-edge appartiene
        HalfEdge* prev = nullptr;       //Puntatore all'half-edge precedente della faccia (senso antiorario)
        HalfEdge* next = nullptr;       //Puntatore all'half-edge successivo della faccia (senso antiorario)

    private:
        friend Diagram;
        typename std::list<HalfEdge>::iterator it;  //iteratore che punta all'HalfEdge nella lista mHalfEdges di tutti gli half-edges
    };
    //Struttura di una Face
    struct Face
    {
        Site* site;                     //Puntatore al sito associato alla faccia
        HalfEdge* outerComponent;       //Puntatore ad uno qualsiasi degli half-edge della faccia (ne basta uno, tanto con i puntatori prev e next possiamo accedere a tutti gli altri)
    };

    //---------------------------------Copy Constructor----------------------------------------------------------------------------------------------------------------
    Diagram(const Diagram& other)
    {
        mSites = other.mSites;
        mFaces = other.mFaces;
        mVertices = other.mVertices;
        mHalfEdges = other.mHalfEdges;
    }
    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
    //---------------------------------Assignment Constructor----------------------------------------------------------------------------------------------------------
    Diagram& operator=(const Diagram& other)
    {
        mSites = other.mSites;
        mFaces = other.mFaces;
        mVertices = other.mVertices;
        mHalfEdges = other.mHalfEdges;

        return *this;
    }
    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------
    //Default Move Constructor
    Diagram(Diagram&&) = default;
    //Default Move Assignment Constructor
    Diagram& operator=(Diagram&&) = default;

    //Metodi Getter pubblici
    //Get vettore dei siti mSites
    const std::vector<Site>& getSites() const
    {
        return mSites;
    }
    //Get puntatore ad un sito specifico del vettore dei siti mSites
    const Site* getSite(std::size_t i) const
    {
        return &mSites[i];
    }
    //Get dimensione del vettore mSites dei siti (= numero di elementi)
    std::size_t getNbSites() const
    {
        return mSites.size();
    }
    //Get vettore della facce mFaces
    const std::vector<Face>& getFaces() const
    {
        return mFaces;
    }
    //Get puntatore ad una faccia specifica del vettore delle facce mFaces
    const Face* getFace(std::size_t i) const
    {
        return &mFaces[i];
    }
    //Get lista dei vertici mVertices
    const std::list<Vertex>& getVertices() const
    {
        return mVertices;
    }
    //Get lista degli half-edges mHalfEdges
    const std::list<HalfEdge>& getHalfEdges() const
    {
        return mHalfEdges;
    }

//added    //////////GETTER POSIZIONE GLOBALE////////////////////
    const Vector2<T> getGlobalPoint() const
    {
        return mGlobalPoint;
    }
//    //////////////////////////////////////////////////////

//added    //Algoritmo di intersezione nel caso di Diagramma con solo sito/faccia centrale --> diagramma deve conicidere con box in modo che il sito stia fermo(densità uniforme) o si muova verso p_t (densità gaussiana)
    void intersect_null(Box<T> box)     //creo come halfedge del diagramma i lati della box e come vertici del diagramma i corner della box
    {
        //Creazione del primo halfedge e salvataggio nella variabile startedge
        auto startedge = createHalfEdge(&mFaces[0]);
        auto halfedge = startedge;

        //Creazione halfedge e vertex/corner
        for(auto side = static_cast<int>(Box<T>::Side::Left); side <= static_cast<int>(Box<T>::Side::Top); ++side){
            //Creazione vertice/corner
            halfedge->destination = createCorner(box, static_cast<typename Box<T>::Side>(side));
            //Gestione halfedge diversi da startedge di partenza
            if(side != static_cast<int>(Box<T>::Side::Top)){
                halfedge->next = createHalfEdge(halfedge->incidentFace);
                halfedge->next->prev = halfedge;
                halfedge->next->origin = halfedge->destination;
                halfedge = halfedge->next;
            }
            //Gestione ultimo halfedge uguale a startedge
            else{
                halfedge->next = startedge;
                halfedge->next->prev = halfedge;
                halfedge->next->origin = halfedge->destination;
            }
        }
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Algoritmo di intersezione con la Intersection Box
    bool intersect(Box<T> box)
    {
        auto success = true;
        auto processedHalfEdges = std::unordered_set<HalfEdge*>();  //set che conterrà gli half-edge già processati (unordered_set solo per prestazioni)
        auto verticesToRemove = std::unordered_set<Vertex*>();  //set che conterrà i vertici da rimuovere (unordered_set solo per prestazioni)
        //Risaliamo innanzitutto agli half-edge tramite il puntatore outerComponent di ogni faccia
        //Ciclo for ripetuto per ogni sito o equivalentemente per ogni faccia
        for (const auto& site : mSites)
        {
            auto halfEdge = site.face->outerComponent;  //salviamo l'half-edge puntato dalla faccia
            auto inside = box.contains(halfEdge->origin->point); //variabile bool: true se il vertice di origine dell'half-edge è interno alla box, false se è esterno
            auto outerComponentDirty = !inside; //variabile bool opposta alla precedente
            auto incomingHalfEdge = static_cast<HalfEdge*>(nullptr); //primo half-edge entrante (resettato ad ogni scansione di una nuova faccia)
            auto outgoingHalfEdge = static_cast<HalfEdge*>(nullptr); //ultimo half-edge uscente (resettato ad ogni scansione di una nuova faccia)
            auto incomingSide = typename Box<T>::Side{};    //side della box interessata dall'half-edge entrante
            auto outgoingSide = typename Box<T>::Side{};    //side della box interessata dall'half-edge uscente
            //Scansione uno ad uno degli half-edge della faccia e controllo intersezioni con box
            do
            {   //Ciclo ripetuto fino all'esaurimento degli half-edge della faccia
                auto intersections = std::array<typename Box<T>::Intersection, 2>{}; //array intersezioni dell'half-edge con la box (massimo 2 vedi immagine colorata quaderno)
                auto nbIntersections = box.getIntersections(halfEdge->origin->point, halfEdge->destination->point, intersections); //restituisce il numero delle intersezioni e salva i p.ti di inteserzione nell'array intersections
                auto nextInside = box.contains(halfEdge->destination->point);   //variabile bool: true se il vertice di destinazione dell'half-edge è interno alla box, false se è esterno
                auto nextHalfEdge = halfEdge->next; //prossimo half-edge concatenato a quello in esame
                //5 casi possibili: (vedi quaderno)
                //Vertice origine e vertice destinazione entrambi fuori dalla box
                if (!inside && !nextInside)
                {
                    //Caso 2. half-edge completamente esterno alla box --> nbIntersections=0
                    if (nbIntersections == 0)
                    {
                        verticesToRemove.emplace(halfEdge->origin); //inseriamo il vertice di origine dell'half-edge nel set dei vertici da rimuovere (solo il vertice di origine perché tanto scansioniamo tutti gli half-edge)
                        removeHalfEdge(halfEdge);   //rimuoviamo l'half-edge
                    }
                    //Caso 5. half-edge attraversa la box 2 volte -->nbIntersections=2
                    else if (nbIntersections == 2)
                    {
                        verticesToRemove.emplace(halfEdge->origin);
                        //Controlliamo che nella lsta degli half-edge processati non ci sia già il suo twin
                        if (processedHalfEdges.find(halfEdge->twin) != processedHalfEdges.end())
                        {   //se lo è
                            halfEdge->origin = halfEdge->twin->destination; //inverto solo l'half-edge tanto poi l'unordered_set rifiuta i doppioni
                            halfEdge->destination = halfEdge->twin->origin; //unordered_set rifiuta i doppioni quindi non verrà aggiunto di nuovo al set degli half-edge già scansionati
                        }
                        else
                        {   //se non è mai stato scansionato
                            halfEdge->origin = createVertex(intersections[0].point);    //vertice di origine del vettorino che bisogna creare (vedi immagine quaderno)
                            halfEdge->destination = createVertex(intersections[1].point);   //vertice di destinazione del vettorino che bisogna creare (vedi immagine quaderno)
                        }
                        if (outgoingHalfEdge != nullptr)    //è ora di linkare il vettorino
                            link(box, outgoingHalfEdge, outgoingSide, halfEdge, intersections[0].side);
                        if (incomingHalfEdge == nullptr)    //prima volta che si becca quella cella
                        {
                           incomingHalfEdge = halfEdge;     //lo marchiamo come edge entrante
                           incomingSide = intersections[0].side;    //prendiamo il side della box corrispondente al vertice 0 come incomingSide per il linkaggio
                        }
                        outgoingHalfEdge = halfEdge;        //lo marchiamo anche come edge uscente
                        outgoingSide = intersections[1].side;   //prendiamo il side della box corrispondente al vertice 1 come outgoingSide per il linkaggio
                        processedHalfEdges.emplace(halfEdge);   //aggiungiamo l'half-edge alla lista degli half-edge processati
                    }
                    //Caso di debug (non dovrebbe mai accadere)
                    else
                        success = false;
                }
                //Caso 3. half-edge è uscente dalla box --> nbIntersections=1
                else if (inside && !nextInside)
                {
                    //Numero intersezioni può essere anche > 1 oltre che = se consideriamo i corner
                    if (nbIntersections >= 1)
                    {   //se nella lista degli half-edge già processati è presente il gemello/twin dell'half-edge in esame
                        if (processedHalfEdges.find(halfEdge->twin) != processedHalfEdges.end())
                            halfEdge->destination = halfEdge->twin->origin; //inverto l'edge, stesso ragionamento(vedi sopra)
                        else    //half-edge mai scansionato
                            halfEdge->destination = createVertex(intersections[0].point);
                        outgoingHalfEdge = halfEdge;    //lo marchiamo come edge uscente
                        outgoingSide = intersections[0].side;   //prendiamo il side della box corrispondente al vertice 1 come outgoingSide per il linkaggio
                        processedHalfEdges.emplace(halfEdge);   //aggiungiamo l'half-edge alla lista degli half-edge processati
                    }
                    //Caso di debug (non dovrebbe mai accadere)
                    else
                        success = false;
                }
                //Caso 4. half-edge è entrante nella box ---> nbIntersections=1
                else if (!inside && nextInside)
                {
                    //Numero intersezioni può essere anche > 1 oltre che = se consideriamo i corner
                    if (nbIntersections >= 1)
                    {
                        verticesToRemove.emplace(halfEdge->origin);
                        //se nella lista degli half-edge già processati è presente il gemello/twin dell'half-edge in esame
                        if (processedHalfEdges.find(halfEdge->twin) != processedHalfEdges.end())
                            halfEdge->origin = halfEdge->twin->destination; //inverto l'edge, stesso ragionamento(vedi sopra)
                        else  //half-edge mai scansionato
                            halfEdge->origin = createVertex(intersections[0].point);
                        if (outgoingHalfEdge != nullptr)    //se l'outgoingHalfEdge della cella è già stato trovato, possiamo linkare
                            link(box, outgoingHalfEdge, outgoingSide, halfEdge, intersections[0].side);
                        if (incomingHalfEdge == nullptr)
                        {
                           incomingHalfEdge = halfEdge;         //lo marchiamo come edge entrante
                           incomingSide = intersections[0].side;  //prendiamo il side della box corrispondente al vertice 0 come incomingSide per il linkaggio
                        }
                        processedHalfEdges.emplace(halfEdge);   //aggiungiamo l'half-edge alla lista degli half-edge processati
                    }
                    //Caso di debug (non dovrebbe mai accadere)
                    else
                        success = false;
                }
                //Update half-edge (per poter scansionare l'half-edge successivo della faccia)
                halfEdge = nextHalfEdge;
                //Update inside bool variable (settando come origine, la destinazione del precedente half-edge)
                inside = nextInside;
            } while (halfEdge != site.face->outerComponent);    //ciclo for termina quando tutti gli half-edge della faccia sono stati analizzati
            //Linkaggio dell'ultimo e del primo half-edge all'interno della box
            if (outerComponentDirty && incomingHalfEdge != nullptr)
                link(box, outgoingHalfEdge, outgoingSide, incomingHalfEdge, incomingSide);
            //Set outerComponent
            if (outerComponentDirty)
                site.face->outerComponent = incomingHalfEdge;
        }
        //Rimozione vertici salvati nell'unordered_set
        for (auto& vertex : verticesToRemove)
            removeVertex(vertex);
        //Return success
        return success;
    }

    //Lloyd's relaxation --> CVT (Centroidal Voronoi Tessellation) Metodo per Diagramma Centralizzato
    std::vector<Vector2<T>> computeLloydRelaxation() const
    {
        auto centroids = std::vector<Vector2<T>>(); //definizione vettore dei siti/generatori
        //Ciclo loop che considera tutte le facce/celle di voronoi una alla volta
        for (const auto& face : mFaces)
        {
            auto area = static_cast<T>(0.0);        //inizializzazione area (double)
            auto centroid = Vector2<T>();           //inizializzazione vettore dei centroidi (vettore di double)
            auto halfEdge = face.outerComponent;    //prendiamo l'half-edge puntato dalla faccia che sta venendo elborata
            //Compute centroid of the face (vedi quaderno per formule)
            do
            {
                auto det = halfEdge->origin->point.getDet(halfEdge->destination->point); //prodotto vettoriale vettore origine e vettore destinazione dell'half-edge considerato
                area += det;    //contributo al calcolo dell'area del poligono
                centroid += (halfEdge->origin->point + halfEdge->destination->point) * det; //contributo al calcolo del centroide
                halfEdge = halfEdge->next;  //passaggio all'half-edge successiva della faccia considerata
            } while (halfEdge != face.outerComponent);  //ciclo do-while continua fino ad esaurimento degli half-edge della faccia considerato
            area *= 0.5;    //area del poligono
            centroid *= 1.0 / (6.0 * area); //centroide del poligono
            centroids.push_back(centroid);  //inserimento nel vettore dei centroidi
        }   //dopodiché si prosegue con la faccia successiva
        return centroids;   //restituisce vettore dei centroidi(che verrà sfruttato come vettore parametro per la funzione di costruzione del Diagram)
    }
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Triangulation
    Triangulation computeTriangulation() const
    {
        auto neighbors = std::vector<std::vector<std::size_t>>(mSites.size());
        for (auto i = std::size_t(0); i < mSites.size(); ++i)
        {
            auto face = mFaces[i];
            auto halfEdge = face.outerComponent;
            while (halfEdge->prev != nullptr)
            {
                halfEdge = halfEdge->prev;
                if (halfEdge == face.outerComponent)
                    break;
            }
            while (halfEdge != nullptr)
            {
                if (halfEdge->twin != nullptr)
                    neighbors[i].push_back(halfEdge->twin->incidentFace->site->index);
                halfEdge = halfEdge->next;
                if (halfEdge == face.outerComponent)
                    break;
            }
        }
        return Triangulation(neighbors);
    }

private:
    //Variabili interne/Containers
    std::vector<Site> mSites;       //vettore di struct Site (siti/generatori)
    std::vector<Face> mFaces;       //vettore di struct Face (celle di Voronoi)
    std::list<Vertex> mVertices;    //lista di struct Vertex (vertici di Voronoi)
    std::list<HalfEdge> mHalfEdges; //lista di struct HalfEdge (half-edges) --> ogni edge di una cella è splittato in 2 half-edge
    Vector2<T> mGlobalPoint;        //coordinate globali iniziali del sito proprietario del diagramma (possiamo fornire al robot una indicazione della sua posizione iniziale globale e AREA_SIZE???)

    //Friendship per rendere accessibile la sezione private alla classe FortuneAlgorithm
    template<typename>
    friend class FortuneAlgorithm;

    //Costruttore Diagramma di Voronoi Centralizzato
    Diagram(const std::vector<Vector2<T>>& points, Box<T> box)
    {   //Usiamo il reserve per evitare riallocazioni della memoria run time che potrebbero incidere negativamente sulle performance dell'algoritmo
        mSites.reserve(points.size());  //Riserviamo uno spazio in memoria per il vettore dei siti pari al numero di siti introdotti
        mFaces.reserve(points.size());  //Riserviamo uno spazio in memoria per la lista delle facce pari al numero di siti introdotti

        //Costruzione DCEL in base ad input forniti
        for (auto i = std::size_t(0); i < points.size(); ++i)   //pre-incremento = più prestazioni
        {   //Inseriamo i siti input nel vettore dei siti --> costruzione mSites e mFaces
            //I siti vengono numerati coerentemente con le posizioni i occupate nel vettore "points" di input e se ne acquisiscono le coordinate x,y sfruttando la notazione associativa ad array
            //-----------ESCLUDI PUNTI FUORI DALLA BOX(centralized)-----------------------------------------------------------------------------
            if(box.contains(points[i])){
                mSites.push_back(Diagram::Site{i, points[i], nullptr});
                //Costruiamo il vettore delle facce mFaces associando i puntatori sito degli elementi Face di mFaces ai siti di mSites
                mFaces.push_back(Diagram::Face{&mSites.back(), nullptr});
                //Associazione dei puntatori faccia degli elementi Site di mSites alle facce corrispondenti di mFaces
                mSites.back().face = &mFaces.back();
            }
        }
    }

    //added //Costruttore Diagramma di Voronoi Decentralizzato
    Diagram(const std::vector<Vector2<T>>& flt_points, Box<T> box, const Vector2<T>& point_global)
    {   //Usiamo il reserve per evitare riallocazioni della memoria run time che potrebbero incidere negativamente sulle performance dell'algoritmo
        mSites.reserve(flt_points.size()+1);  //Riserviamo uno spazio in memoria per il vettore dei siti pari al numero di siti introdotti (fondamentale +1 per tenere conto del mainSite)
        mFaces.reserve(flt_points.size()+1);  //Riserviamo uno spazio in memoria per la lista delle facce pari al numero di siti introdotti (fondamentale +1 per tenere conto del mainSite)

        //registro coordinate globali in mGlobalPoint
        mGlobalPoint = point_global;
        //inserire il mainSite
        auto mainSite = Vector2<T>{0.0, 0.0};

        //Marchiamolo come sito index 0 così da averlo sempre in testa (e poterlo sfruttare in seguito per il calcolo del centroide)
        mSites.push_back(Diagram::Site{0, mainSite, nullptr});
        mFaces.push_back(Diagram::Face{&mSites.back(), nullptr});
        mSites.back().face = &mFaces.back();

        //Costruzione DCEL in base ad input forniti
        for (auto i = std::size_t(0); i < flt_points.size(); ++i)   //pre-incremento = più prestazioni
        {   //Inseriamo i siti input nel vettore dei siti --> costruzione mSites e mFaces
            mSites.push_back(Diagram::Site{i+1, flt_points[i], nullptr});
            //Costruiamo il vettore delle facce mFaces associando i puntatori sito degli elementi Face di mFaces ai siti di mSites
            mFaces.push_back(Diagram::Face{&mSites.back(), nullptr});
            //Associazione dei puntatori faccia degli elementi Site di mSites alle facce corrispondenti di mFaces
            mSites.back().face = &mFaces.back();
        }

        //debug print Find Neighbours
        if (DEBUG >= 1)
        {
            std::cout << "\nSiti Diagramma: " << std::endl;
            for(const auto& site : mSites){
                std::cout << "Sito " << site.index  << ": " << site.point << std::endl;
            }
        }
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Get puntatore ad un sito specifico del vettore dei siti mSites
    Site* getSite(std::size_t i)
    {
        return &mSites[i];
    }
    //Get puntatore ad una faccia specifica del vettore delle facce mFaces
    Face* getFace(std::size_t i)
    {
        return &mFaces[i];
    }

    //Creazione di un Vertex e inserimento nella lista mVertices
    Vertex* createVertex(Vector2<T> point)  //parametro: point(x,y)
    {
        mVertices.emplace_back();  //Creazione nuovo elemento Vertex di mVertices
        mVertices.back().point = point; //Definizione coordinate in base alle coordinate input della funzione
        mVertices.back().it = std::prev(mVertices.end());  //Sistemo l'iteratore sull'elemento/vertice appena introdotto-->prev(mVertices.end()) equivale a mVertices.end()-1, operazione necessaria perché end punta sempre al nullptr
        return &mVertices.back();   //restituisce il puntatore all'elemento Vertex appena creato e inserito
    }
    //Creazione vertici corner della box
    Vertex* createCorner(Box<T> box, typename Box<T>::Side side)
    {   //Funzione varia a seconda del side della box specificato come parametro
        switch (side)
        {
            case Box<T>::Side::Left:
                return createVertex(Vector2<T>(box.left, box.top));
            case Box<T>::Side::Bottom:
                return createVertex(Vector2<T>(box.left, box.bottom));
            case Box<T>::Side::Right:
                return createVertex(Vector2<T>(box.right, box.bottom));
            case Box<T>::Side::Top:
                return createVertex(Vector2<T>(box.right, box.top));
            default:
                return nullptr;
        }
    }
    //Creazione di un HalfEdge e inserimento nella lista mHalfEdges
    HalfEdge* createHalfEdge(Face* face)   //parametro: puntatore ad una faccia incidente all'halfedge
    {
        mHalfEdges.emplace_back();  //Creazione nuovo elemento HalfEdge di mHalfEdges
        mHalfEdges.back().incidentFace = face;  //Definizione della faccia incidente, ovvero la face input della funzione
        mHalfEdges.back().it = std::prev(mHalfEdges.end()); //Sistemo l'iteratore sull'elemento/half-edge appena introdotto-->prev(mHalfEdges.end()) equivale a mHalfEdges.end()-1, operazione necessaria perché end punta sempre al nullptr
        //Nel caso in cui la faccia non punti ancora a nessun half-edge, facciamola puntare all'half-edge appena creato
        if (face->outerComponent == nullptr)
            face->outerComponent = &mHalfEdges.back();  //si evita così che ci possano essere facce che non puntano a nessun half-edge/outerComponent
        return &mHalfEdges.back();  //restituisce il puntatore all'elemento HalfEdge appena creato e inserito
    }

    //Metodo di linkaggio Diagramma di Voronoi con Bounding Box
    void link(Box<T> box, HalfEdge* start, typename Box<T>::Side startSide, HalfEdge* end, typename Box<T>::Side endSide)
    {
        auto halfEdge = start; //salvo nella variabile halfedge il puntatore all'half edge iniziale per poterlo utilizzare nel codice
        auto side = static_cast<int>(startSide); //startSide è una variabile della enum class (LEFT,RIGHT,BOTTOM,UP) quindi faccio il cast trasformandola in numero intero
        while (side != static_cast<int>(endSide))   //finchè side non arriva al valore endSide
        {
            side = (side + 1) % 4;  //corrisponde a side++ ma quando arriva a 3 ristarta da 0 anziché incrementare al valore 4. In questo modo c'è perfetta corrispondenza con la enum
            halfEdge->next = createHalfEdge(start->incidentFace);   //creo un nuovo half-edge incidente alla stessa faccia di quello di start
            halfEdge->next->prev = halfEdge;        //aggancio il nuovo creato ad half-edge
            halfEdge->next->origin = halfEdge->destination; //completo l'aggancio settando l'origine del nuovo come destinazione del vecchio
            halfEdge->next->destination = createCorner(box, static_cast<typename Box<T>::Side>(side));  //creo un corner della bounding box nel nodo di arrivo del nuovo half-edge
            halfEdge = halfEdge->next;  //itero in avanti prendendo come half-edge il nuovo half-edge finché non arrivo all'endside
        }
        //Gestione ultimo tratto (vedi quaderno)
        halfEdge->next = createHalfEdge(start->incidentFace);
        halfEdge->next->prev = halfEdge;
        end->prev = halfEdge->next;
        halfEdge->next->next = end;
        halfEdge->next->origin = halfEdge->destination;
        halfEdge->next->destination = end->origin;
    }

    //Metodi remove
    //Rimozione di un elemento Vertex dalla lista mVertices
    void removeVertex(Vertex* vertex) //parametro: vertex da rimuovere
    {   //metodo erase richiede l'iteratore all'elemento da rimuovere
        //metodo erase ridimensiona automaticamente la list mVertices dopo la rimozione
        mVertices.erase(vertex->it);
    }
    //Rimozione di un elemento HalfEdge dalla lista mHalfEdges
    void removeHalfEdge(HalfEdge* halfEdge) //parametro: halfEdge da rimuovere
    {   //metodo erase richiede l'iteratore all'elemento da rimuovere
        //metodo erase ridimensiona automaticamente la list mVertices dopo la rimozione
        mHalfEdges.erase(halfEdge->it);
    }
};
