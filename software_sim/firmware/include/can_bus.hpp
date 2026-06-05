#pragma once 
#include <cstdint> 
#include "message.hpp"

// ~14.3 Hz x 60s = ~857 msgs + ~143 buffer
//constexpr instead of const to fix size at compile time (safety)
constexpr uint32_t BUS_CAPACITY=1000; //Size 32 bc STM32's native arch is 32. Else CPU would need to convert 16 bit vals to 32, and then back to 16 post computation.

//use template type to have one class that will work for both SensorMessage and ControlMessage
template <typename T>
class CANBus{
public: 
    //constructor: initializes the queue indexes to 0
    CANBus() {
        head=0;
        tail=0;
        count=0; 
    }

    //WRITE function
    //Adds a message to the tail of the queue. Returns true if successful, false if queue is full. 
    bool send(const T& msg)
    {
        if(is_full())
        {
            return false; //queue's full so can't add anymore messages 
        }
        buffer[tail]=msg;
        tail=(tail+1)%BUS_CAPACITY; //circular loop around math 
        count++;
        return true;
    }

    //READ function
    //Takes a message off the head of the queue. Returns true if read was successful. 
    //passing in param by reference bc returning large structures by value introduces unnecessary CPU copying overhead
    bool receive(T& output_msg)
    {
        if(is_empty())
        {
            return false; //queue's empty so there's nothing to read 
        }
        output_msg=buffer[head];
        head=(head+1)%BUS_CAPACITY; //circular loop around math
        count--;
        return true;
    }

    //status check functions (all read only):
    bool is_empty() const
    {
        return count==0;
    }

    bool is_full() const
    {
        return count==BUS_CAPACITY;
    }

    uint32_t size() const
    {
        return count;
    }

private:
    T buffer[BUS_CAPACITY]; //fixed size array to prevent dynamic memory allocation
    uint32_t head;
    uint32_t tail;
    uint32_t count;
};
