#ifndef LIST_H
#define LIST_H

#include <lattice_planner/state.h>

#define LISTSIZE 2e7

namespace lattice_planner {

struct ListElement {
    State *listState;
    struct ListElement *prev;
    struct ListElement *next;
};

class List {
    public:
        
        List() {
            firstElement = NULL;
            lastElement = NULL;
            cur_size = 0;
        }

        ~List() {}

        bool isEmpty() {
            return (cur_size == 0);
        }

        bool isFull() {
            return (cur_size >= LISTSIZE);
        }

        bool inList(State *state) {
            return (state->listElement != NULL);
        }

        void makeEmpty() {
            while (firstElement != NULL) {
                deleteElement(firstElement->listState);
            }
        }

        void insertElement(State *state) {
            if (cur_size >= LISTSIZE) {
                ROS_ERROR("List::insertElement: list is 'full'");
            }
            if (state->listElement != NULL) {
                ROS_ERROR("List::insertElement: state already in list");
            }
            ListElement *newElement = (ListElement*)malloc(sizeof(ListElement));
            newElement->listState = state;
            newElement->prev = NULL;
            newElement->next = firstElement;
            if (firstElement != NULL) {
                firstElement->prev = newElement;
            }
            firstElement = newElement;
            if (lastElement == NULL) {
                lastElement = newElement;
            }
            state->listElement = newElement;
            cur_size++;
        }

        void deleteElement(State *state) {
            if (cur_size == 0 || state->listElement == NULL) {
                ROS_ERROR("List::deleteElement: list does not contain state");
            }
            if (state->listElement->prev != NULL &&
                state->listElement->next != NULL) {
                state->listElement->prev->next = state->listElement->next;
                state->listElement->next->prev = state->listElement->prev;
            } else if (state->listElement->prev != NULL) {
                state->listElement->prev->next = NULL;
                lastElement = state->listElement->prev;
            } else if (state->listElement->next != NULL) {
                state->listElement->next->prev = NULL;
                firstElement = state->listElement->next;
            } else {
                firstElement = NULL;
                lastElement = NULL;
            }
            free(state->listElement);
            state->listElement = NULL;
            cur_size--;
        }

        State *getFirst() {
            if (firstElement == NULL) {
                return NULL;
            } else {
                return firstElement->listState;
            }
        }

        State *getLast() {
            if (lastElement == NULL) {
                return NULL;
            } else {
                return lastElement->listState;
            }
        }

        State *getNext(State *state) {
            if (state->listElement->next == NULL) {
                return NULL;
            } else {
                return state->listElement->next->listState;
            }
        }

        ListElement *firstElement;
        ListElement *lastElement;
        int cur_size;
};

} /* namespace lattice_planner */

#endif /* LIST_H */
