#pragma once
// My includes
#include "Vettore.h"
#include "Diagram.h"
#include "Arc.h"
#include "Utility.h"

//Classe Beachline --> RB-Tree Data Strcuture
template<typename T>
class Beachline
{
public:
    //Costruttore Beachline/Albero
    Beachline() : mNil(new Arc<T>), mRoot(mNil) //arco aggiunto come foglia rappresenta inizialmente anche la radice
    {
        mNil->color = Arc<T>::Color::Black;     //sia nodi Nil che nodo Root devono essere neri (proprietà 2,3)
    }
    //Distruttore Beachline/Albero
    ~Beachline()
    {
        free(mRoot);    //funzione di eliminazione completa albero a partire dal nodo root
        delete mNil;    //recupero memoria
    }


    //Rimozione Copy Constructor
    Beachline(const Beachline&) = delete;
    Beachline& operator=(const Beachline&) = delete;
    //Move Constructor
    Beachline(Beachline&& other)
    {
        mNil = other.mNil;
        mRoot = other.mRoot;
        other.mNil = nullptr;
        other.mRoot = nullptr;
    }
    //Assign Constructor
    Beachline& operator=(Beachline&& other)
    {
        free(mRoot);
        delete mNil;
        mNil = other.mNil;
        mRoot = other.mRoot;
        other.mNil = nullptr;
        other.mRoot = nullptr;
    }

    //Metodi pubblici
    //Creazione di un arco di parabola dato il sito
    Arc<T>* createArc(typename Diagram<T>::Site* site, typename Arc<T>::Side side = Arc<T>::Side::Left)
    {   //Crea puntatore ad un nuovo blocco arco
        return new Arc<T>{mNil, mNil, mNil, site, nullptr, nullptr, nullptr, mNil, mNil, Arc<T>::Color::Red, side};
    }
    //Verifica della presenza di almeno un arco/nodo nella Beachline
    bool isEmpty() const
    {
        return isNil(mRoot); //true: albero vuoto, il nodo radice è esso stesso una foglia
    }
    //Verificare se un nodo/arco è una foglia del tree
    bool isNil(const Arc<T>* x) const
    {
        return x == mNil;   //true: l'arco x è una foglia che non punta a nulla (Nil)
    }
    //Settare un arco x come root
    void setRoot(Arc<T>* x)
    {
        mRoot = x;  //eguaglio i puntatori
        mRoot->color = Arc<T>::Color::Black;  //colore nodo root: nero
    }
    //Ricavare l'arco più a sinistra sulla Beachline
    Arc<T>* getLeftmostArc() const
    {
        auto x = mRoot;   //la ricerca parte dal nodo root
        //Scansione di tutti gli archi/nodi passando ogni volta all'arco precedente dell'arco corrente
        while (!isNil(x->prev))
            x = x->prev;
        return x;   //restituisco l'arco più a sinistra
    }
    //Localizzare l'arco su cui si innesta il nuovo arco associato al sito specificato come parametro, considerando la posizione corrente della sweep line (l)
    Arc<T>* locateArcAbove(const Vector2<T>& point, T l) const
    {
        auto node = mRoot;      //la ricerca parte dal nodo root
        auto found = false;     //flag booleano di ricerca
        //Ciclo di ricerca continua fintanto che non viene trovato l'arco desiderato
        while (!found)
        {
            auto breakpointLeft = -std::numeric_limits<T>::infinity();
            auto breakpointRight = std::numeric_limits<T>::infinity();
            if (!isNil(node->prev)) //se il precedente della beachline non è una foglia, calcoliamo la coordinata x di intersezione dei due
                breakpointLeft =  computeBreakpoint(node->prev->site->point, node->site->point, l, node->prev->side);
            if (!isNil(node->next)) //se il successivo della beachline non è una foglia, calcoliamo la coordinata x di intersezione dei due
                breakpointRight = computeBreakpoint(node->site->point, node->next->site->point, l, node->next->side);
            if (point.x < breakpointLeft)
                node = node->left;  //ci si sposta nel nodo leftchild
            else if (point.x > breakpointRight)
                node = node->right; //ci si sposta nel nodo rightchild
            else
                found = true;   //nodo trovato
        }
        return node;
    }
    //Inserimento di un arco y prima di un arco x
    void insertBefore(Arc<T>* x, Arc<T>* y)
    {
        //Step 1: Individuare la posizione in cui inserire il nodo y
        //Non esiste un leftchild dell'arco x
        if (isNil(x->left))
        {
            x->left = y;
            y->parent = x;
        }
        //Esiste già un leftchild dell'arco x
        else
        {
            x->prev->right = y;
            y->parent = x->prev;
        }
        //Step 2: Settare i puntatori DCEL
        y->prev = x->prev;
        if (!isNil(y->prev))
            y->prev->next = y;
        y->next = x;
        x->prev = y;
        //Step 3: Ribilanciare l'albero
        insertFixup(y);
    }
    //Inserimento di un arco y prima di un arco x
    void insertAfter(Arc<T>* x, Arc<T>* y)
    {
        //Step 1: Individuare la posizione in cui inserire il nodo y
        //Non esiste un rightchild dell'arco x
        if (isNil(x->right))
        {
            x->right = y;
            y->parent = x;
        }
        //Esiste già un rightchild dell'arco x
        else
        {
            x->next->left = y;
            y->parent = x->next;
        }
        //Step 2: Settare i puntatori DCEL
        y->next = x->next;
        if (!isNil(y->next))
            y->next->prev = y;
        y->prev = x;
        x->next = y;
        //Step 3: Ribilanciare l'albero
        insertFixup(y);
    }
    //Sostituzione di un arco/nodo x con un altro arco/nodo y: quindi quando un arc viene sostituito da un middleArc, il middleArc diventa parent di leftArc e rightArc
    void replace(Arc<T>* x, Arc<T>* y)
    {
        transplant(x, y);           //transplant sottoalbero di radice x con sottoalbero di radice y
        y->left = x->left;          //y eredita il leftchild di x
        y->right = x->right;        //y eredita il rightchild di x
        if (!isNil(y->left))
            y->left->parent = y;    //aggancio il leftchild a y nel caso x ne avesse uno
        if (!isNil(y->right))
            y->right->parent = y;   //aggancio il rightchild a y nel caso x ne avesse uno
        //Update puntatori preve e next DCEL
        y->prev = x->prev;
        y->next = x->next;
        if (!isNil(y->prev))
            y->prev->next = y;
        if (!isNil(y->next))
            y->next->prev = y;
        y->color = x->color;        //y eredita il colore di x
    }
    //Rimozione di un arco/nodo della beachline (vedi 03graphs.pdf slides 113-114)
    void remove(Arc<T>* z)
    {
        auto y = z;
        auto yOriginalColor = y->color;
        auto x = mNil;      //mNil serve solo come inizializzazione per x
        //Case(a): il leftchild di z è una foglia (Nil)
        if (isNil(z->left))
        {   //In questo caso si effettua un seplice transplant con il rightchild di z
            x = z->right;
            transplant(z, z->right);
        }
        //Case(b): il rightchild di z è una foglia (Nil)
        else if (isNil(z->right))
        {   //In questo caso
            x = z->left;
            transplant(z, z->left);
        }
        //Case(c && d): né il leftchild né il rightchild di z sono foglie
        else
        {
            y = minimum(z->right); //puntatore y al nodo leftchild minimo del rightchild di z
            yOriginalColor = y->color;
            x = y->right;          //puntatore x al nodo rightchild di y
            //Case(c only): se non esistono altri nodi tra z e y (perché il minimum del rightchild di z è il rightchild di z stesso)
            if (y->parent == z)
                x->parent = y;     //aggancio la foglia x a y
            //Case(d only): esistono altri nodi tra z e y
            else
            {
                transplant(y, y->right); //si effettua il transplant del sottoalbero di y con quello del suo rightchild
                y->right = z->right;     //il rightchild di z diventa rightchild di y
                y->right->parent = y;    //completo l'aggancio a y
            }
            //Case(c && d)
            transplant(z, y);       //si effettua il transplant dei sottoalberi
            y->left = z->left;      //il leftchild di z viene agganciato a y come suo leftchild
            y->left->parent = y;    //completo l'aggancio a y
            y->color = z->color;
        }
        //Ribilanciamento dell'albero per ripristono proprietà RB-Tree
        if (yOriginalColor == Arc<T>::Color::Black)
            removeFixup(x);
        //Update dei puntatori prev e next per bypassare l'arco eliminato
        if (!isNil(z->prev))
            z->prev->next = z->next;    //next del precedente di z = next di z
        if (!isNil(z->next))
            z->next->prev = z->prev;    //prev del successivo di z = prev di z
    }
    //Funzione di stampa pubblica dell'RB-Tree
    std::ostream& print(std::ostream& os) const
    {
        std::cout << "Root: ";
        return printArc(os, mRoot); //printArc esegue una stampa ricorsiva in modo da stampare tutto l'RB-Tree a partire dal nodo Root
//        auto arc = getLeftmostArc();
//        while (!isNil(arc))
//        {
//            os << arc->site->index << ' ';
//            arc = arc->next;
//        }
//        return os;
    }

private:
    //Variabili interne
    Arc<T>* mNil;   //Nodo foglia
    Arc<T>* mRoot;  //Nodo root

    //Ricerca del nodo discendente sinistro di x minimo
    Arc<T>* minimum(Arc<T>* x) const
    {   //Scansione di tutti i leftchild del nodo x finché non si giunge alla foglia più a sinistra
        while (!isNil(x->left))
            x = x->left;
        return x;
    }
    //Trapianto/scambio del sottoalbero del nodo u con il sottoalbero del nodo v
    void transplant(Arc<T>* u, Arc<T>* v)   //u e v sono i nodi radice dei sottoalberi da scambiare
    {
        if (isNil(u->parent))   //caso in cui la radice del sotto-albero è la radice (root) dell'albero intero
            mRoot = v;          //semplice ridefinizione del nodo root
        else if (u == u->parent->left)  //caso in cui u è un leftchild di suo padre
            u->parent->left = v;        //assegno indice v all'attuale u
        else
            u->parent->right = v;       //assegno indice v all'attuale u
        //Si può procedere ora allo scambio dei due sottoalberi
        v->parent = u->parent;
    }

    //Ribilanciamento albero a seguito dell'inserimento di un nuovo arco/nodo (rosso)
    void insertFixup(Arc<T>* z)
    {   //vedi libro 2 pag.281-282
        //Ripristino proprietà 4 RB-Tree (se un nodo è rosso, i suoi figli sono entrambi neri)
        while (z->parent->color == Arc<T>::Color::Red)
        {   //Il padre di z è un leftchild
            if (z->parent == z->parent->parent->left)
            {
                auto y = z->parent->parent->right;  //y: zio di z
                //Case 1: y è rosso
                if (y->color == Arc<T>::Color::Red)
                {   //Recolor
                    z->parent->color = Arc<T>::Color::Black;
                    y->color = Arc<T>::Color::Black;
                    z->parent->parent->color = Arc<T>::Color::Red;
                    z = z->parent->parent;  //sposto puntatore z
                }
                //y è nero
                else
                {
                    //Case 2: z è un rightchild
                    if (z == z->parent->right)
                    {
                        z = z->parent;  //sposto puntatore z
                        leftRotate(z);  //left-rotation
                    }
                    //Case 3: z è ora un leftchild
                    //Recolor
                    z->parent->color = Arc<T>::Color::Black;
                    z->parent->parent->color = Arc<T>::Color::Red;
                    rightRotate(z->parent->parent); //right-rotation
                }
            }
            //Il padre di z è un right child
            else
            {
                auto y = z->parent->parent->left;   //y: zio di z
                //Case 1: y è rosso
                if (y->color == Arc<T>::Color::Red)
                {   //Recolor
                    z->parent->color = Arc<T>::Color::Black;
                    y->color = Arc<T>::Color::Black;
                    z->parent->parent->color = Arc<T>::Color::Red;
                    z = z->parent->parent;  //sposto puntatore z
                }
                //y è nero
                else
                {
                    //Case 2: z è un leftchild
                    if (z == z->parent->left)
                    {
                        z = z->parent;  //sposto puntatore
                        rightRotate(z); //right-rotation
                    }
                    //Case 3: z è ora un rightchild
                    //Recolor
                    z->parent->color = Arc<T>::Color::Black;
                    z->parent->parent->color = Arc<T>::Color::Red;
                    leftRotate(z->parent->parent);  //left-rotation
                }
            }
        }
        //Ripristino proprietà 2 RB-Tree (nodo root sempre nero)
        mRoot->color = Arc<T>::Color::Black;
    }
    //Ribilanciamento albero a seguito dell'eliminazione di un arco/nodo
    void removeFixup(Arc<T>* x)
    {   //vedi 03graph.pdf
        while (x != mRoot && x->color == Arc<T>::Color::Black)  //ciclo di fixaggio continua fintanto che x è di colore nero e non è il nodo root
        {   //x è un leftchild di suo padre
            if (x == x->parent->left)
            {
                auto w = x->parent->right;  //w: fratello di x
                //Case 1: il fratello di x è rosso
                if (w->color == Arc<T>::Color::Red)
                {   //Recolor di w e del parent di x
                    w->color = Arc<T>::Color::Black;
                    x->parent->color = Arc<T>::Color::Red;
                    leftRotate(x->parent);  //left-rotation del parent di x
                    w = x->parent->right;   //setto puntatore w sul rightchild del parent di x
                }
                //Ora w è nero
                //Case 2: il leftchild di w è nero e il rightchild di w è nero
                if (w->left->color == Arc<T>::Color::Black && w->right->color == Arc<T>::Color::Black)
                {   //Recolor di w
                    w->color = Arc<T>::Color::Red;
                    x = x->parent;  //setto puntatore x sul parent di x
                }
                //il leftchild di w è rosso o il rightchild di w è rosso o entrambe le cose
                else
                {
                    //Case 3: il rightchild di w è nero
                    if (w->right->color == Arc<T>::Color::Black)
                    {   //Recolor
                        w->left->color = Arc<T>::Color::Black;
                        w->color = Arc<T>::Color::Red;
                        rightRotate(w); //right-rotation
                        w = x->parent->right;   //setto puntatore w sul rightchild del parent di x
                    }
                    //Case 4: il rightchild di w è rosso
                    //Recolor
                    w->color = x->parent->color;
                    x->parent->color = Arc<T>::Color::Black;
                    w->right->color = Arc<T>::Color::Black;
                    leftRotate(x->parent);  //left-rotation
                    x = mRoot;  //settando x come root termina il loop
                }
            }
            //x è un rightchild di suo padre (speculare)
            else
            {
                auto w = x->parent->left;
                //Case 1
                if (w->color == Arc<T>::Color::Red)
                {
                    w->color = Arc<T>::Color::Black;
                    x->parent->color = Arc<T>::Color::Red;
                    rightRotate(x->parent);
                    w = x->parent->left;
                }
                //Case 2
                if (w->left->color == Arc<T>::Color::Black && w->right->color == Arc<T>::Color::Black)
                {
                    w->color = Arc<T>::Color::Red;
                    x = x->parent;
                }
                else
                {
                    //Case 3
                    if (w->left->color == Arc<T>::Color::Black)
                    {
                        w->right->color = Arc<T>::Color::Black;
                        w->color = Arc<T>::Color::Red;
                        leftRotate(w);
                        w = x->parent->left;
                    }
                    // Case 4
                    w->color = x->parent->color;
                    x->parent->color = Arc<T>::Color::Black;
                    w->left->color = Arc<T>::Color::Black;
                    rightRotate(x->parent);
                    x = mRoot;
                }
            }
        }
        x->color = Arc<T>::Color::Black;    //x è il root quindi deve essere nero
    }

    //Rotazione a sinistra di un arco x - procedura standard (pag.278 cap13 libro2)
    void leftRotate(Arc<T>* x)
    {
        auto y = x->right;          //setto puntatore
        x->right = y->left;         //faccio diventare il leftchild di y, un rightchild di x
        if (!isNil(y->left))
            y->left->parent = x;    //aggancio il sottoalbero
        y->parent = x->parent;      //faccio diventare il parent di x, il parent di y
        if (isNil(x->parent))
            mRoot = y;
        else if (x->parent->left == x)
            x->parent->left = y;    //y diventa il leftchild del parent di x
        else
            x->parent->right = y;   //oppure y diventa il rightchild del parent di x
        y->left = x;                //x diventa il leftchild di y
        x->parent = y;              //il parent di x è ora y
    }
    //Rotazione a destra di un arco x - procedura standard (pag.278 cap13 libro2)
    void rightRotate(Arc<T>* y)     //stessa cosa ma speculare
    {
        auto x = y->left;
        y->left = x->right;
        if (!isNil(x->right))
            x->right->parent = y;
        x->parent = y->parent;
        if (isNil(y->parent))
            mRoot = x;
        else if (y->parent->left == y)
            y->parent->left = x;
        else
            y->parent->right = x;
        x->right = y;
        y->parent = x;
    }
    //Calcolo coordinata x di intersezione/breakpoint tra due archi dati i siti associati point1, point2 e la posizione y della sweepline(l)
    T computeBreakpoint(const Vector2<T>& point1, const Vector2<T>& point2, T l, typename Arc<T>::Side side) const
    {   //Semplifichiamo la trattazione usando delle variabili per le coordinate x e y dei due siti
        auto x1 = point1.x, y1 = point1.y, x2 = point2.x, y2 = point2.y;
        //Se i due archi hanno la stessa curvatura (poiché le y dei siti sono circa uguali)
        if (almostEqual(y1, y2))
        {
            //Caso 1. Breakpoint di intersezione si trova a metà tra le coordinate x dei due punti
            if (x1 < x2)
                return (x1 + x2) / 2;
            //Caso 2. Breakpoint di intersezione è all'infinito (-oo o +oo a seconda del side dell'arco)
            else
                return side == Arc<T>::Side::Left ?
                    -std::numeric_limits<T>::infinity() :
                    std::numeric_limits<T>::infinity();
        }
        //Se è un arco è ancora un raggio, ovvero la coordinata y del sito dell'arco coincide con la coordinata y corrente della sweep line
        if (almostEqual(y1, l))
            return x1;
        if (almostEqual(y2, l))
            return x2;
        //In tutti gli altri casi ci sono due intersezioni --> calcolo con formule analitiche parabola (vedi altro quaderno)
        auto d1 = 1.0 / (2.0 * (y1 - l));   //parametrizzazzione comoda per cacolare a,b,c
        auto d2 = 1.0 / (2.0 * (y2 - l));   //parametrizzazzione comoda per cacolare a,b,c
        auto a = d1 - d2;
        auto b = 2.0 * (x2 * d2 - x1 * d1);
        auto c = (y1 * y1 + x1 * x1 - l * l) * d1 - (y2 * y2 + x2 * x2 - l * l) * d2;
        auto delta = b * b - 4.0 * a * c;
        return (-b + std::sqrt(delta)) / (2.0 * a);
    }
    //Eliminazione sottoalbero del nodo x(lavora in concomitanza con il distruttore)
    void free(Arc<T>* x)
    {   //processo ricorsivo fino alla completa rimozione di tutti i nodi
        if (isNil(x))    //nodo x è già una foglia
            return;      //stop
        else             //nodo x non è una foglia
        {
            free(x->left);  //processo ricorsivo per il leftchild di x
            free(x->right); //processo ricorsivo per il rightchild di x
            delete x;       //eliminazione del nodo x stesso
        }
    }
    //Funzione stampa dell'RB-TREE (Stampa ricorsiva di tutto il sottoalbero di un arco)
    std::ostream& printArc(std::ostream& os, const Arc<T>* arc, std::string tabs = "") const
    {   //Stampiamo l'indice del sito associato all'arco
        os << arc->site->index << std::endl;// ' ' << arc->leftHalfEdge << ' ' << arc->rightHalfEdge << std::endl;
        if (!isNil(arc->left)){
            std::cout << "Leftchild di " << arc->site->index << ": ";
            printArc(os, arc->left);//, tabs + '\t');
        }
        if (!isNil(arc->right)){
            std::cout << "Rightchild di " << arc->site->index << ": ";
            printArc(os, arc->right);//, tabs + '\t');
        }
        return os;
    }
};

////Overload operatore di stream/stampa per un oggetto Beachline
template<typename T>
std::ostream& operator<<(std::ostream& os, const Beachline<T>& beachline)
{
    return beachline.print(os);
}
