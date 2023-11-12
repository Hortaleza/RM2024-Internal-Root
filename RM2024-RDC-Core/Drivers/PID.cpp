#include "PID.hpp"

float abs(float n)
{
    if (n >= 0)
        return n;
    return -n;
}

double abs(double n)
{
    if (n >= 0)
        return n;
    return -n;
}

#if USE_PID
namespace Control
{

/*This is where you implement the PID algorithm*/

// float PID::update(float target, float measurement, float dt)
// {
//     /*=====================================================================*/
//     // Your implementation of the PID algorithm begins here
//     /*=====================================================================*/
//     error = target - measurement;  // error = target - current

//     pOut = Kp * error;  // pOut = Kp * error

//     iOut += Ki * error * dt;  // iOut = integral(Ki * error)

//     dOut = Kd * (error - lastError) / dt;  // dOut = derivative(Kd * error)

//     output = pOut + iOut + dOut;  // output = pOut + iOut + dOut

//     lastError = error;

//     /*=====================================================================*/
//     // Your implementation of the PID algorithm ends here
//     /*=====================================================================*/
//     return this->output;  // You need to give your user the output for every update
// }

float PID::update(float target, float measurement, float dt)
{
    // @todo: change the time unit to be 1ms
    integral = 0;
    error = target - measurement;  // Calculate the error
    //if (abs(error) <=  200)
    //{
        if (output > maxoutput)
        {
            if(error < 0)
            {
                integral = error * dt;
            }
        }
        else if (output < minoutput)
        {
            if (error > 0)
            {
                integral = error * dt;
            }
        }
        else
        {
            integral = error * dt;
        }
    //}
    currentfilter = a * previousfilter + (1 - a) * (error-lastError);
    previousfilter = currentfilter;
    pOut = Kp * error;             // Calculate the P term
    iOut += Ki * integral;    // Calculate the I term
    dOut = Kd * currentfilter/dt;   // Calculate the D term
    output = pOut + iOut + dOut ;  // The output is the sum of all terms
    lastError = error;     // Update the last error
    /*=====================================================================*/
    // Your implementation of the PID algorithm ends here
    /*=====================================================================*/
    return this->output;  // You need to give your user the output for every update
}

void PID::clear()
{
    lastError = 0;
    iOut           = 0; // should change this actually ....
    previousfilter = 0;
}

}  // namespace Control
#endif
