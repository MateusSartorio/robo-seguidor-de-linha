#pragma once

template<int STACK_MAX_SIZE>
class Stack
{
private:
  int arr[STACK_MAX_SIZE];
  int top;

public:
  Stack()
  {
    top = -1;
  } // Initialize top to -1 to indicate an empty stack

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

  int topElement()
  {
    if (!isEmpty())
    {
      return arr[top];
    }
    else
    {
      return -1; // In this example, we consider -1 as an invalid value.
    }
  }

  void clear()
  {
    while (!isEmpty())
    {
      pop();
    }
  }
};