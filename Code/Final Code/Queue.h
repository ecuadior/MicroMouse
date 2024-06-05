#ifndef QUEUE_H
#define QUEUE_H

// Define a Node structure for each element in the queue
template<typename T>
struct Node {
    T data;
    Node<T>* next;
};

// Define the Queue class
template<typename T>
class Queue {
private:
    Node<T>* front;
    Node<T>* rear;

public:
    // Constructor
    Queue() : front(nullptr), rear(nullptr) {}

    // Destructor
    ~Queue() {
        while (!isEmpty()) {
            dequeue();
        }
    }

    // Check if the queue is empty
    bool isEmpty() const {
        return front == nullptr;
    }

    // Enqueue an element to the queue
    void enqueue(const T& value) {
        Node<T>* newNode = new Node<T>{value, nullptr};
        if (isEmpty()) {
            front = rear = newNode;
        } else {
            rear->next = newNode;
            rear = newNode;
        }
    }

    // Dequeue an element from the queue
    T dequeue() {
        if (isEmpty()) {
            // Handle underflow
            // You might want to throw an exception or return a default value
        } else {
            Node<T>* temp = front;
            T value = front->data;
            front = front->next;
            delete temp;
            if (front == nullptr) {
                rear = nullptr;
            }
            return value;
        }
    }
};

#endif // QUEUE_H