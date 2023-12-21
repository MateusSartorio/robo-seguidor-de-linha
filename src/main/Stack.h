#pragma once

/**
 * @brief Classe Stack implementa estrutura de dados do tipo pilha para inteiros positivos
 *
 * @tparam STACK_MAX_SIZE
 */
template <int STACK_MAX_SIZE>
class Stack
{
private:
  int arr[STACK_MAX_SIZE];
  int top;

public:
  /**
   * @brief Constroi um novo objeto da classe Stack
   *
   */
  Stack()
  {
    top = -1;
  }

  /**
   * @brief Retorna true caso a pilha esteja vazia
   *
   * @return bool
   */
  bool isEmpty()
  {
    return (top == -1);
  }

  /**
   * @brief Retorna true caso a pilha esteja cheia
   *
   * @return bool
   */
  bool isFull()
  {
    return (top == STACK_MAX_SIZE - 1);
  }

  /**
   * @brief Insere elemento no top
   *
   * @param element
   */
  void push(int element)
  {
    if (!isFull())
    {
      top++;
      arr[top] = element;
    }
  }

  /**
   * @brief Remove elemento do top
   */
  void pop()
  {
    if (!isEmpty())
    {
      int poppedElement = arr[top];
      top--;
    }
  }

  /**
   * @brief Retorna elemento no topo da pilha
   *
   * @return int
   */
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

  /**
   * @brief Limpa a pilha
   */
  void clear()
  {
    while (!isEmpty())
    {
      pop();
    }
  }
};