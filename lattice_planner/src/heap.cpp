#include <lattice_planner/heap.h>

/********************************************************************
 * Heap implementation. It's generic enough to work regardless of how
 * states are being implemented. Leaning fairly heavily on the SBPL 
 * implementation. Memory being allocated by demand. I think the rest 
 * is self explanitory as far as heaps go.
 ********************************************************************/

namespace lattice_planner {

//#define MAX 1e10

Heap::Heap() {
    cur_size = 0;
    allocated = HEAPSIZE_INIT;
    heap = new HeapElement[allocated];
}

Heap::~Heap() {
    for (int i=1; i<=cur_size; i++) {
        heap[i].heapState->heapIndex = 0;
    }
    delete[] heap;
}

void Heap::percolateDown(int hole, HeapElement tmp) {
    int child;
    if (cur_size != 0) {
        for (; 2*hole <= cur_size; hole = child) {
            child = 2*hole;

            if (child != cur_size && heap[child+1].key < heap[child].key) {
                child++;
            }
            if (heap[child].key < tmp.key) {
                heap[hole] = heap[child];
                heap[hole].heapState->heapIndex = hole;
            } else {
                break;
            }
        }
        heap[hole] = tmp;
        heap[hole].heapState->heapIndex = hole;
    }
}

void Heap::percolateUp(int hole, HeapElement tmp) {
    if (cur_size != 0) {
        for (; hole > 1 && tmp.key < heap[hole/2].key; hole /= 2) {
            heap[hole] = heap[hole/2];
            heap[hole].heapState->heapIndex = hole;
        }
        heap[hole] = tmp;
        heap[hole].heapState->heapIndex = hole;
    }
}

void Heap::percolateUpOrDown(int hole, HeapElement tmp) {
    if (cur_size != 0) {
        if (hole > 1 && heap[hole/2].key > tmp.key) {
            percolateUp(hole, tmp);
        } else {
            percolateDown(hole, tmp);
        }
    }
}

bool Heap::isEmpty() {
    return cur_size == 0;
}

bool Heap::isFull() {
    return cur_size == HEAPSIZE - 1;
}

bool Heap::inHeap(State *state) {
    return (state->heapIndex != 0);
}

void Heap::makeEmpty() {
    for (int i=1; i<=cur_size; i++) {
        heap[i].heapState->heapIndex = 0;
    }
    cur_size = 0;
}

void Heap::makeHeap() {
    for (int i=cur_size/2; i> 0; i--) {
        percolateDown(i, heap[i]);
    }
}

void Heap::insertElement(State *state, double key) {
    sizeCheck();
    if (state->heapIndex != 0 ) {
        ROS_ERROR("Heap::insertElement: state already in heap");
        return;
    }
    HeapElement tmp;
    tmp.heapState = state;
    tmp.key = key;
    percolateUp(++cur_size, tmp);
}

void Heap::deleteElement(State *state) {
    if (state->heapIndex == 0) {
        ROS_ERROR("Heap::deleteElement: state not in heap");
        return;
    }
    percolateUpOrDown(state->heapIndex, heap[cur_size--]);
    state->heapIndex = 0;
}

void Heap::updateElement(State *state, double new_key) {
    if (state->heapIndex == 0) {
        ROS_ERROR("Heap::updateElement: state not in heap");
        return;
    }
    if (heap[state->heapIndex].key != new_key) {
        heap[state->heapIndex].key = new_key;
        percolateUpOrDown(state->heapIndex, heap[state->heapIndex]);
    }
}

State* Heap::getMinElement() {
    if (cur_size == 0) {
        ROS_ERROR("Heap::getMinElement: heap empty");
        return NULL;
    }
    return heap[1].heapState;
}

State* Heap::getMinElement(double &ret_key) {
    if (cur_size == 0) {
        ROS_ERROR("Heap::getMinElement: heap empty");
        return NULL;
    }
    ret_key = heap[1].key;
    return heap[1].heapState;
}

State* Heap::deleteMinElement() {
    if (cur_size == 0) {
        ROS_ERROR("Heap::deleteMinElement: heap empty");
        return NULL;
    }
    State *state;
    state = heap[1].heapState;
    state->heapIndex = 0;
    percolateDown(1, heap[cur_size--]);
    return state;
}

double Heap::getMinKey() {
    double ret;
    if (cur_size == 0) {
        ret = MAX;
    } else {
        ret = heap[1].key;
    }
    return ret;
}

double Heap::getKey(State *state) {
    if (state->heapIndex == 0) {
        ROS_ERROR("Heap::getKey: state not in heap");
        return 0;
    }
    return heap[state->heapIndex].key;
}

void Heap::growHeap() {
    HeapElement *newHeap;
    
    ROS_INFO("growing heap from %d", allocated);
    allocated = 2*allocated;
    if (allocated > HEAPSIZE) {
        allocated = HEAPSIZE;
    }
    ROS_INFO("to %d", allocated);

    newHeap = new HeapElement[allocated];

    for (int i=0; i<=cur_size; i++) {
        newHeap[i] = heap[i];
    }
    delete[] heap;
    heap = newHeap;
}

void Heap::sizeCheck() {
    if (isFull()) {
        ROS_WARN("Heap is full");
    } else if (cur_size == allocated - 1) {
        growHeap();
    }
}


} /* namepsace lattice_planner */
