#pragma once

template<int STACK_MAX_SIZE>
class Stack
{
private:
  int arr[STACK_MAX_SIZE];
  int top;

public:
  // Initializa topo com -1 para indicar uma pilha vazia
  Stack()
  {
    top = -1;
  }

  bool isEmpty()
  {
    return (top == -1);
  }

  bool isFull()
  {
    return (top == STACK_MAX_SIZE - 1);
  }

  void push(int element)
  {
    if (!isFull())
    {
      top++;
      arr[top] = element;
    }
  }

  void pop()
  {
    if (!isEmpty())
    {
      int poppedElement = arr[top];
      top--;
    }
  }

  // Retorna -1 caso a pilha esteja vazia
  int topElement()
  {
    if (!isEmpty())
    {
      return arr[top];
    }
    else
    {
      return -1;
    }
  }

  // Limpa a pilha
  void clear()
  {
    while (!isEmpty())
    {
      pop();
    }
  }
};