# Rigid 2D transformation Library
A library for handling transformations in SE(2).

# Conceptual Questions
1. What is the difference between a class and a struct in C++?
	1. Members of a class are private by default and members of a struct are public by default.
	2. When deriving a strcut from a class/strcut, default access specifier for a base class/struct is public. And deriving a class, default access specifier is private. 

2. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specic C++ core guidelines in your answer)?
	1. C.2: Use class if the class has an invariant; use struct if the data members can vary independently. The x and y component of Vector2D can vary independently. Nevertheless, the theta, ctheta, stheta, x and y component of Transform2D depend on each other since Vetcor2D can be represented in any frame and Transfrom2D represents the relationship between 2 frames.
	2. C.8: Use class rather than struct if any member is non-public. Readability reason. To make it clear that something is being hidden/abstracted. The theta, ctheta, stheta x and y component in Transfrom2D are private. The x and y component of Vector2D are public.

3. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?
	
	C.46: By default, declare single-argument constructors explicit to avoid unintended conversions. The explicit constructors in Transform2D are single-argument ones.

4. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   - Which of the methods would you implement and why?
   
	1. Design1: Create a normalize class. 
	Pros: The normal vector would behave like a std::vector(array of doubles) making it a concrete type.
	Cons: It is not necessary to represent in class since the data types will likly to be visible(public). A class takes more work  and is not easily as readable as a struct.
	2. Design2: Create a normalize vector struct that inherits from Vector2D struct.
	Pros: This method builds a relatioship between objects.
	Cons: May make the code confusing for reading.
	3. Design3: Create a normalize struct to store normal vectors and a helper function in rigid2d namespace to compute normal vectors.
	Pros: It is easy and convenient to use and it does not build on any methods within Transform2D class.
	Cons: If the normal vectors wants to be private, a class should be implemented instead.
	
	I will choose Design3 since it is easier to use and does not require any methods from Transform2D class.  

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not (Refer to C++ core guidelines in your answer)?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer.
   
	Con.2: By default, make member functions const.
	Reason:A member function should be marked const unless it changes the object’s observable state. This gives a more precise statement of design intent, better readability, more errors caught by the compiler, and sometimes more optimization opportunities. The Transform2D::inv() invert a Transform2D without modifying it and create a new Transform2D. It does not change the observable state. However, the Transform2D::operator*=() modifies the passed Transform2D by multiplying with another Transform2D. 
   
   
