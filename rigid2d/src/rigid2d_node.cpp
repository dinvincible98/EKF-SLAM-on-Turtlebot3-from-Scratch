#include<iostream>
#include"rigid2d/rigid2d.hpp"

using namespace rigid2d;
using namespace std;


int main()
{

    // inputs
    Transform2D Tab, Tbc;
    Vector2D v;
    Twist2D twist;
    char frame;

    // Compute these outputs
    Transform2D Tba,Tcb,Tac,Tca;
    
    cout<<"------------------------------------"<<endl;
    cout<<"Tab"<<endl;
    cin>>Tab;

    cout<<"-------------------------------------"<<endl;
    cout<<"Tbc"<<endl;
    cin>>Tbc;

    // Compute inverse
    Tba = Tab.inv();
    Tcb = Tbc.inv();

    // Compute dot product
    Tac = Tab * Tbc;
    Tca = Tac.inv();


    cout<<"-------------------------------------"<<endl;
    
    // Tab
    cout<<"Tab: ";
    cout<<Tab;
    // Tba
    cout<<"Tba: ";
    cout<<Tba;
    // Tbc
    cout<<"Tbc: ";
    cout<<Tbc;
    // Tcb
    cout<<"Tcb: ";
    cout<<Tcb;
    // Tac
    cout<<"Tac: ";
    cout<<Tac;
    // Tca
    cout<<"Tca: ";
    cout<<Tca; 

    cout<<"-------------------------------------"<<endl;

    // Vector in frame
    cout<<"Enter a vector: "<<endl;
    cin>>v;
    
    cout<<"Enter a frame: "<<endl;
    cin>>frame;

    if (frame=='a')
    {
        // In frame a
        cout<<"Vector in frame a: ";
        cout<<v;

        //In frame b
        cout<<"Vector in frame b: " ;
        Vector2D vb = Tba(v);
        cout<<vb;

        //In c frame
        cout<<"Vector in frame c: " ;
        Vector2D vc = Tca(v);
        cout<<vc;
    }
    else if(frame=='b')
    {
        // In frame a
        cout<<"Vector in frame a: ";
        Vector2D va = Tab(v);
        cout<<va;

        //In frame b
        cout<<"Vector in frame b: " ;
        cout<<v;

        //In c frame
        cout<<"Vector in frame c: " ;
        Vector2D vc = Tcb(v);
        cout<<vc;
    }
    else if(frame=='c')
    {
        // In frame a
        cout<<"Vector in frame a: ";
        Vector2D va = Tac(v);
        cout<<va;

        //In frame b
        cout<<"Vector in frame b: " ;
        Vector2D vb = Tbc(v);
        cout<<vb;

        //In c frame
        cout<<"Vector in frame c: " ;
        cout<<v;
    }

    cout<<"------------------------------------"<<endl;

    // Twist in frame
    cout<<"Enter a twist: "<<endl;
    cin>>twist;
    
    if (frame == 'a')
    {
        // In frame a
        cout<<"Twist in frame a: ";
        cout<<twist;

        // In frame b
        cout<<"Twist in frame b: ";
        Twist2D tb = Tba(twist);
        cout<<tb;

        // In frame c
        cout<<"Twist in frame c: ";
        Twist2D tc = Tca(twist);
        cout<<tc;
    }

    else if (frame == 'b')
    {
        // In frame a
        cout<<"Twist in frame a: ";
        Twist2D ta = Tab(twist);
        cout<<ta;

        // In frame b
        cout<<"Twist in frame b: ";
        cout<<twist;

        // In frame c
        cout<<"Twist in frame c: ";
        Twist2D tc = Tcb(twist);
        cout<<tc;
    }

    else if (frame == 'c')
    {
        // In frame a
        cout<<"Twist in frame a: ";
        Twist2D ta = Tac(twist);
        cout<<ta;

        // In frame b
        cout<<"Twist in frame b: ";
        Twist2D tb = Tbc(twist);
        cout<<tb;

        // In frame c
        cout<<"Twist in frame c: ";
        cout<<twist;
    }

    cout<<"-------------------------------------"<<endl;


    return 0;
}