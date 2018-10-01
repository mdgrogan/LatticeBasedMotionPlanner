#ifndef HEAP_H
#define HEAP_H

#include <lattice_planner/state.h>

#define HEAPSIZE 2e7
#define HEAPSIZE_INIT 10000

namespace lattice_planner {

struct HeapElement {
    State *heapState;
    double key;
};

class Heap {
    public:

        Heap();
        ~Heap();
        bool isEmpty();
        bool isFull();
        bool inHeap(State *state);
        void makeEmpty();
        void makeHeap();
        void insertElement(State *state, double key);
        void deleteElement(State *state);
        void updateElement(State *state, double new_key);
        State *getMinElement();
        State *getMinElement(double &ret_key);
        State *deleteMinElement();
        double getMinKey();
        double getKey(State *state);

        HeapElement *heap;
        int cur_size;
        int allocated;

    private:

        void percolateDown(int hole, HeapElement tmp);
        void percolateUp(int hole, HeapElement tmp);
        void percolateUpOrDown(int hole, HeapElement tmp);

        void growHeap();
        void sizeCheck();
};


} /* namespace lattice_planner */

#endif /* HEAP_H */
