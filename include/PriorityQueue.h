#pragma once
// STL
#include <ostream>
#include <vector>
#include <memory>

//Classe PriorityQueue --> MAX-HEAP Data Structure
template<typename T>
class PriorityQueue
{
public:
    //Costruttore
    PriorityQueue()
    {}

    //Verifica presenza di almeno un elemento/evento da processare
    bool isEmpty() const
    {   //metodo per controllare se la queue è vuota
        return mElements.empty();   //true = container vuoto
    }

    //N.B. Il container vector è ottimizzato per le rimozioni in coda quindi gli eventi da rimuovere vengono spostati in coda prima di essere rimossi con il pop_back
    //Rimozione dell'evento top del container + ripristino ordinamento decrescente y
    std::unique_ptr<T> pop()
    {
        swap(0, mElements.size() - 1);          //lo sposto in fondo al container, scambiandone temporaneamente la posizione con l'ultimo nodo
        auto top = std::move(mElements.back()); //creo iteratore che punta all'ultimo elemento
        mElements.pop_back();                   //lo elimino
        siftDown(0);                            //ripristino condizione di MAX-HEAP riportando in posizione top il nodo con il valore maggiore(siftDown a partire dal nodo root)
        return top;                             //restituisco l'iteratore al primo evento (queue top: y maggiore)
    }
    //Inserimento di un nuovo evento + ripristino ordinamento decrescente y
    void push(std::unique_ptr<T>&& elem)
    {
        elem->index = mElements.size();          //assegnamento index all'evento in base alla dimensione corrente del container
        mElements.emplace_back(std::move(elem)); //inserimento del nuovo evento
        siftUp(mElements.size() - 1);            //ripristino condizione di MAX HEAP tramite siftUp del nodo introdotto che parte dal fondo della queue
    }

    //Update/Ripristino condizione di MAX-HEAP (incorpora siftUp e siftDown in un'unica funzione pubblica)
    void update(std::size_t i)
    {
        auto parent = getParent(i); //parent=indice del parent di i
        if (parent >= 0 && *mElements[parent] < *mElements[i])
            siftUp(i);      //siftUp eseguita se nodo i ha valore maggiore del suo parent
        else
            siftDown(i);    //siftDown eseguita se nodo i ha valore minore dei suoi nodi figli
    }
    //Rimozione di un nodo arbitrario i anche se intermedio (motivo principale costruzione del MAX-HEAP)
    void remove(std::size_t i)
    {
        swap(i, mElements.size() - 1);  //lo sposto in fondo al container, scambiandone temporaneamente la posizione con l'ultimo nodo
        mElements.pop_back();           //lo elimino
        //Nodo i prima era in fondo quindi per ripristinare la condizione di MAX-HEAP occorre eseguire un update() (o siftDown()) a meno che non si trattasse del penultimo
        if (i < mElements.size())
            update(i);
    }

    //Funzione stampa dell'evento top della PriorityQueue (funzione associata all'overload dell'operatore di stream in Event.h)
    std::ostream& print(std::ostream& os, std::size_t i = 0, std::string tabs = "") const
    {
        if (i < mElements.size())
        {
            os << tabs << *mElements[i] << std::endl;
        }
        return os;
    }

private:
    //Container base del MAX_HEAP: vector di puntatori ad oggetti Event (uno smart pointer per ogni evento)
    std::vector<std::unique_ptr<T>> mElements;

    //Metodi Getter
    //Get indice del parent di un elemento/nodo del container
    int getParent(int i) const
    {
        return (i + 1) / 2 - 1;
    }
    //Get indice del leftchild di un elemento/nodo del container
    std::size_t getLeftChild(std::size_t i) const
    {
        return 2 * (i + 1) - 1;
    }
    //Get indice del rightchild di un elemento/nodo del container
    std::size_t getRightChild(std::size_t i) const
    {
        return 2 * (i + 1);
    }
//-----------------------------------------------------------------------------------------------------------------------------------
//METODI FONDAMENTALI per ORDINAMENTO e RIPRISTINO CONDIZIONE DI MAX-HEAP
    //Spostamento in basso di un elemento i fino a portarlo nella posizione corretta(ordine decrescente)
    void siftDown(std::size_t i)    //utile per ripristino la condizione di MAX-HEAP dopo inserimento di un nuovo evento/nodo
    {
        auto left = getLeftChild(i);    //left=indice leftchild di i
        auto right = getRightChild(i);  //right=indice rightchild di i
        auto j = i;                     //salvataggio indice i in j
        //Risistemo albero HEAP nel caso in cui il nodo j abbia valore minore del leftchild di i
        if (left < mElements.size() && *mElements[j] < *mElements[left])
            j = left;   //marchio con indice j il leftchild di i
        //A questo punto controllo che non sia invece il rightchild ad avere il valore maggiore
        if (right < mElements.size() && *mElements[j] < *mElements[right])
            j = right;  //marchio con indice j il leftchild di i
        //Se il nodo j non coincide con il nodo i: o nodo i non è una foglia o nodo i non rispetta l'ordine decrescente
        if (j != i)
        {
            swap(i, j);     //scambio di elementi i e j
            siftDown(j);    //processo ricorsivo fino a quando l'elemento i non arriva nella posizione corretta della queue
        }
    }
    //Spostamento in alto di un elemento i fino a portarlo nella posizione corretta(ordine decrescente)
    void siftUp(std::size_t i)  //utile per ripristino la condizione di MAX-HEAP dopo rimozione di un evento/nodo
    {
        auto parent = getParent(i);     //parent=indice del parent di i
        //Risistemo albero HEAP nel caso in cui il suo nodo i abbia valore maggiore di quello del suo parent
        if (parent >= 0 && *mElements[parent] < *mElements[i])
        {
            swap(i, parent);    //scambio di elemento i con il suo parent
            siftUp(parent);     //processo ricorsivo fino a quando l'elemento i non arriva nella posizione corretta della queue o diventa il root dell'albero
        }
    }
//-----------------------------------------------------------------------------------------------------------------------------------
    //Scambio elemento di indice i con elemento di indice j
    inline void swap(std::size_t i, std::size_t j)
    {
        std::swap(mElements[i], mElements[j]);  //scambio dei valori degli elementi di indici i e j
        mElements[i]->index = i;    //riassegnazione indice corretto
        mElements[j]->index = j;    //riassegnazione indice corretto
    }
};

//Overload operatore di stream/stampa per un oggetto PriorityQueue
template <typename T>
std::ostream& operator<<(std::ostream& os, const PriorityQueue<T>& queue)
{
    return queue.print(os);
}
