/* $Id$
 *
 * Contains classes to represent targets in the behavior
 */

#ifndef __COMMON_HEADER_CPP
#error "This file is meant to be included through common_header.cpp."
#endif

// Rectangle types
#define TARGET_TYPE_NULL    0
#define TARGET_TYPE_RED     1
#define TARGET_TYPE_WHITE   2
#define TARGET_TYPE_GREEN   3
#define TARGET_TYPE_BLUE    7
#define TARGET_TYPE_PURPLE  9

// Specifiable color types
#define TARGET_TYPE_CIRCLE 10
#define TARGET_TYPE_SQUARE 11

// Arc types
#define ARC_TYPE_RED		5
#define ARC_TYPE_WHITE		6
#define ARC_TYPE_BLUE		8
#define ARC_TYPE_GREEN		13
#define ARC_TYPE_PURPLE		14
#define ARC_TYPE_ORANGE		15

// Moving dots target
#define TARGET_TYPE_MOVING_DOTS 12

/***************************************************
 * Target (Parent class)
 ***************************************************/

/**
 * Target parent class.
 * Abstract class that represents all of the different targets
 * that can be used by the behavior.
 */
class Target {
public:
	/**
	 * Copies the target to the outputs array.
	 * This is a true virtual function that must be implemented
	 * for each subclass.  It's purpose is to convert the internal 
	 * data of the target to the list of five real_Ts that is necessary
	 * to send to the graphics program.
	 * @param u the output buffer to write to.
	 * @param offset the location to begin writing.
	 */
	virtual void copyToOutputs(real_T *u, int offset) = 0;

	/**
	 * Determines if the specified point within the target.
	 * This is a pure virtual function that must be implemented
	 * for each subclass.  It's purpose is to indicate whether 
	 * the specified point is within the bounds of this particular
	 * target.
	 * @param x the x coordinate of the point.
	 * @param y the y coordinate of the point.
	 * @return true if the specified point is within the bounds of 
	 * of the target, false otherwise.
	 */
	virtual bool cursorInTarget(double x, double y) = 0;

	/**
	 * Determines if the specified point within the target.
	 * This is a pure virtual function that must be implemented
	 * for each subclass.  It's purpose is to indicate whether 
	 * the specified point is within the bounds of this particular
	 * target.
	 * @param p a Point containg the x,y coordinate to check
	 * @return true if the specified point is within the bounds of 
	 * of the target, false otherwise.
	 */
	virtual bool cursorInTarget(Point p) = 0;
    
    virtual bool voltageInTarget(float targetStaircase) = 0;

	/* See definition below */
	static int Color(int red, int green, int blue);
};

/**
 * Returns an int (which can be cast to real_T) that specifies a color 
 * and can be sent to the graphics computer with the draw instruction.
 * @param r the red value from 0 to 255.
 * @param g the green value from 0 to 255.
 * @param b the blue value from 0 to 255.
 * @return an integer color code for the requested color.
 */
int Target::Color(int r, int g, int b) {
	return ( 256*256*r + 256*g + b );
}



/***************************************************
 * RectangleTarget
 ***************************************************/

/**
 * A rectangular target of an arbitrary type.
 * There are a number of target types that generate rectangular targets
 * of different colors or with different glyphs.  This class represents
 * all of these target types setting the `type` variable will determine
 * which target type is requested from the graphics.
 */
class RectangleTarget : public Target {
public: 
	RectangleTarget();
	RectangleTarget(double left, double top, double right, double bottom, int type);
	void copyToOutputs(real_T *u, int offset);
	bool cursorInTarget(double x, double y);
	bool cursorInTarget(Point p);

	/**
	 * Left edge of the target.
	 */
	double left;

	/**
	 * Right edge of the target.
	 */
	double right;

	/**
	 * Upper edge of the target.
	 */
	double top;

	/**
	 * Lower edge of the target.
	 */
	double bottom;

	/**
	 * The target type to be requested from the graphics computer.
	 */
	double type;
};

/**
 * Default constructor sets all values to zero.
 */
RectangleTarget::RectangleTarget() {
	this->left = 0.0;
	this->right = 0.0;
	this->bottom = 0.0; 
	this->top = 0.0;
	this->type = TARGET_TYPE_NULL;
}

/**
 * Creates a RectangleTarget with the requested boundaries and type.
 * @param left the left edge of the target.
 * @param top the top edge of the target.
 * @param right the right edge of the target.
 * @param bottom the bottom edge of the target.
 * @param type the type of target.
 */
RectangleTarget::RectangleTarget(double left, double top, double right, double bottom, int type) {
	this->left = left;
	this->right = right;
	this->bottom = bottom; 
	this->top = top;
	this->type = type;
}

/**
 * Determines if the specified point within the target.
 * This function will return whether the specified point is
 * within the target.
 * @param x the x coordinate of the point.
 * @param y the y coordinate of the point.
 * @return true if the specified point is within the bounds of 
 * of the target, false otherwise.
 */
bool RectangleTarget::cursorInTarget(double x, double y) {
	return ( (x > this->left) && (x < this->right) && 
		(y > this->bottom) && (y < this->top) );
}

/**
 * Determines if the specified point within the target.
 * Determines if the specified point within the target.
 * This function will return whether the specified point is
 * within the target.
 * @param p a Point containg the x,y coordinate to check
 * @return true if the specified point is within the bounds of 
 * of the target, false otherwise.
 */
bool RectangleTarget::cursorInTarget(Point p) {
	return this->cursorInTarget(p.x, p.y);
}

/**
 * Copies the target to the outputs array.
 * This function will the internal 
 * data of the target to the list of five real_Ts that is necessary
 * to send to the graphics program. This function is called from
 * Behavior::writeOutputs and you should never have to call it 
 * directly.
 * @param u the output buffer to write to.
 * @param offset the location to begin writing.
 */
void RectangleTarget::copyToOutputs(real_T *u, int offset) {
	u[0+offset] = this->type;
	u[1+offset] = this->left;
	u[2+offset] = this->top;
	u[3+offset] = this->right;
	u[4+offset] = this->bottom;
}

/***************************************************
 * ArcTarget
 ***************************************************/

/**
 * An arc target of an arbitrary type.
 * There are a number of target types that generate arc targets
 * of different colors or with different glyphs.  This class represents
 * all of these target types setting the `type` variable will determine
 * which target type is requested from the graphics.
 */
class ArcTarget : public Target {
public: 
	ArcTarget();
	ArcTarget(double r, double theta, double span, double height, int type);
	void copyToOutputs(real_T *u, int offset);
	bool cursorInTarget(double x, double y);
	bool cursorInTarget(Point p);

	/**
	 * Radius to middle of target.
	 */
	double r;

	/**
	 * Angle to middle of target (rad).
	 */
	double theta;

	/**
	 * Angle width of target
	 */
	double span;

	/**
	 * Radius width of target.
	 */
	double height;

	/**
	 * The target type to be requested from the graphics computer.
	 */
	double type;

	double tx1;
	double ty1;
	double tx2;
	double ty2;
	double trsqL;
	double trsqH;
};

/**
 * Default constructor sets all values to zero.
 */
ArcTarget::ArcTarget() {
	this->r = 0.0;
	this->theta = 0.0;
	this->span = 0.0; 
	this->height = 0.0;
	this->type = TARGET_TYPE_NULL;
	this->tx1 = 0.0;
	this->ty1 = 0.0;
	this->tx2 = 0.0;
	this->ty2 = 0.0;
	this->trsqL = 0.0;
	this->trsqH = 0.0;
}

/**
 * Creates an ArcTarget with the requested boundaries and type.
 * @param r radius to middle of target.
 * @param theta angle to middle of target.
 * @param span the angle of separation between target edges.
 * @param height the difference in radius between target edges.
 * @param type the type of target.
 */
ArcTarget::ArcTarget(double r, double theta, double span, double height, int type) {

	this->r = r;
	this->theta = theta;
	this->span = span; 
	this->height = height;
	this->type = type;

	this->tx1 = (this->r + this->height*0.5)*cos(this->theta-0.5*this->span);
	this->ty1 = (this->r + this->height*0.5)*sin(this->theta-0.5*this->span);
	this->tx2 = (this->r - this->height*0.5)*cos(this->theta+0.5*this->span);
	this->ty2 = (this->r - this->height*0.5)*sin(this->theta+0.5*this->span);
	this->trsqL = (this->r - this->height*0.5)*(this->r - this->height*0.5);
	this->trsqH = (this->r + this->height*0.5)*(this->r + this->height*0.5);
}

/**
 * Determines if the specified point within the target.
 * This function will return whether the specified point is
 * within the target.
 * @param x the x coordinate of the point.
 * @param y the y coordinate of the point.
 * @return true if the specified point is within the bounds of 
 * of the target, false otherwise.
 */
bool ArcTarget::cursorInTarget(double x, double y) {

	double rsq;
	double tx1,ty1,tx2,ty2, trsqL,trsqH;

	tx1 = (this->r + this->height*0.5)*cos(this->theta-0.5*this->span);
	ty1 = (this->r + this->height*0.5)*sin(this->theta-0.5*this->span);
	tx2 = (this->r - this->height*0.5)*cos(this->theta+0.5*this->span);
	ty2 = (this->r - this->height*0.5)*sin(this->theta+0.5*this->span);
	trsqL = (this->r - this->height*0.5)*(this->r - this->height*0.5);
	trsqH = (this->r + this->height*0.5)*(this->r + this->height*0.5);

	rsq = x*x + y*y;

	return ( /* distance criterion */ (rsq > trsqL && rsq < trsqH) &&
		     /* angle criterion */    ( x*ty1-y*tx1 < 0 && 
									    x*ty2-y*tx2 > 0 ) );

}

/**
 * Determines if the specified point within the target.
 * Determines if the specified point within the target.
 * This function will return whether the specified point is
 * within the target.
 * @param p a Point containg the x,y coordinate to check
 * @return true if the specified point is within the bounds of 
 * of the target, false otherwise.
 */
bool ArcTarget::cursorInTarget(Point p) {
	return this->cursorInTarget(p.x, p.y);
}

/**
 * Copies the target to the outputs array.
 * This function will copy the internal 
 * data of the target to the list of five real_Ts that is necessary
 * to send to the graphics program. This function is called from
 * Behavior::writeOutputs and you should never have to call it 
 * directly.
 * @param u the output buffer to write to.
 * @param offset the location to begin writing.
 */
void ArcTarget::copyToOutputs(real_T *u, int offset) {
	u[0+offset] = this->type;
	u[1+offset] = (this->r + this->height*0.5)*cos(this->theta-0.5*this->span);
	u[2+offset] = (this->r + this->height*0.5)*sin(this->theta-0.5*this->span);
	u[3+offset] = (this->r - this->height*0.5)*cos(this->theta+0.5*this->span);
	u[4+offset] = (this->r - this->height*0.5)*sin(this->theta+0.5*this->span);
}


/***************************************************
 * CircleTarget
 ***************************************************/

/**
 * A circle target.
 * Represents a circular target of an arbitrary color. The
 * graphics program can draw any color circle target requested.
 */
class CircleTarget : public Target {
public:
	CircleTarget();
	CircleTarget(double centerX, double centerY, double radius, int color);
	void copyToOutputs(real_T *u, int offset);
	bool cursorInTarget(double x, double y);
	bool cursorInTarget(Point p);

	/** The x coordinate of the center of the circle. */
	double centerX;

	/** The y coordinate of the center of the circle. */
	double centerY;

	/** The radius of the circle. */
	double radius;

	/** The color to draw the circle target. */
	int color;
};

/**
 * Default constructor sets all values to zero.
 */
CircleTarget::CircleTarget() {
	this->centerX = 0.0;
	this->centerY = 0.0; 
	this->radius = 0.0;
	this->color = 0;
}

/**
 * Creates a CircleTarget with the requested position, size, and color.
 * @param centerX the x coordinate of the center of the target.
 * @param centerY the y coordinate of the center of the target.
 * @param radius the radius of the target.
 * @param color the color to draw the target.
 */
CircleTarget::CircleTarget(double centerX, double centerY, double radius, int color) {
	this->centerX = centerX;
	this->centerY = centerY; 
	this->radius = radius;
	this->color = color;
}

/**
 * Determines if the specified point within the target.
 * This function will return whether the specified point is
 * within the target.
 * @param x the x coordinate of the point.
 * @param y the y coordinate of the point.
 * @return true if the specified point is within the bounds of 
 * of the target, false otherwise.
 */
bool CircleTarget::cursorInTarget(double x, double y) {
	return ( sqrt( (this->centerX-x)*(this->centerX-x) + (this->centerY-y)*(this->centerY-y) ) < radius );
}

/**
 * Determines if the specified point within the target.
 * Determines if the specified point within the target.
 * This function will return whether the specified point is
 * within the target.
 * @param p a Point containg the x,y coordinate to check
 * @return true if the specified point is within the bounds of 
 * of the target, false otherwise.
 */
bool CircleTarget::cursorInTarget(Point p) {
	return this->cursorInTarget(p.x, p.y);
}

/**
 * Copies the target to the outputs array.
 * This function will the internal 
 * data of the target to the list of five real_Ts that is necessary
 * to send to the graphics program. This function is called from
 * Behavior::writeOutputs and you should never have to call it 
 * directly.
 * @param u the output buffer to write to.
 * @param offset the location to begin writing.
 */
void CircleTarget::copyToOutputs(real_T *u, int offset) {
	u[0+offset] = (real_T)(TARGET_TYPE_CIRCLE);
	u[1+offset] = (real_T)(centerX);
	u[2+offset] = (real_T)(centerY);
	u[3+offset] = (real_T)(centerX + radius);
	u[4+offset] = (real_T)(color);
}


/***************************************************
 * SquareTarget
 ***************************************************/

/**
 * A square target.
 * Represents a square target of an arbitrary color. The
 * graphics program can draw any color square target requested.
 */
class SquareTarget : public Target {
public:
	SquareTarget();
	SquareTarget(double centerX, double centerY, double width, int color);
	void copyToOutputs(real_T *u, int offset);
	bool cursorInTarget(double x, double y);
	bool cursorInTarget(Point p);

	/** The x coordinate of the center of the square target */
	double centerX;

	/** The y coordinate of the center of the square target */
	double centerY;

	/** The width (and height) of the square) */
	double width;

	/** The color to draw the square target. */
    int color;
};

/**
 * Default constructor sets all values to zero.
 */
SquareTarget::SquareTarget() {
	this->centerX = 0.0;
	this->centerY = 0.0;
	this->width = 0.0;
	this->color = 0;
}

/**
 * Creates a SquareTarget with the requested position, size, and color.
 * @param centerX the x coordinate of the center of the target.
 * @param centerY the y coordinate of the center of the target.
 * @param width the width and height of the square target.
 * @param color the color to draw the target.
 */
SquareTarget::SquareTarget(double centerX, double centerY, double width, int color) {
	this->centerX = centerX;
	this->centerY = centerY;
	this->width = width;
	this->color = color;
}

/**
 * Determines if the specified point within the target.
 * This function will return whether the specified point is
 * within the target.
 * @param x the x coordinate of the point.
 * @param y the y coordinate of the point.
 * @return true if the specified point is within the bounds of 
 * of the target, false otherwise.
 */
bool SquareTarget::cursorInTarget(double x, double y) {
	return ( (x > centerX - width/2) && (x < centerX + width/2) && 
		(y > centerY - width/2) && (y < centerY + width/2) );
}

/**
 * Determines if the specified point within the target.
 * Determines if the specified point within the target.
 * This function will return whether the specified point is
 * within the target.
 * @param p a Point containg the x,y coordinate to check
 * @return true if the specified point is within the bounds of 
 * of the target, false otherwise.
 */
bool SquareTarget::cursorInTarget(Point p) {
	return this->cursorInTarget(p.x, p.y);
}

/**
 * Copies the target to the outputs array.
 * This function will the internal 
 * data of the target to the list of five real_Ts that is necessary
 * to send to the graphics program. This function is called from
 * Behavior::writeOutputs and you should never have to call it 
 * directly.
 * @param u the output buffer to write to.
 * @param offset the location to begin writing.
 */
void SquareTarget::copyToOutputs(real_T *u, int offset) {
	u[0+offset] = (real_T)(TARGET_TYPE_SQUARE);
	u[1+offset] = (real_T)(centerX - width / 2);
	u[2+offset] = (real_T)(centerY + width / 2);
	u[3+offset] = (real_T)(centerX + width / 2);
	u[4+offset] = (real_T)this->color;
}


/***************************************************
 * MovingDotsTargetA
 ***************************************************/

/**
 * First part of a moving dots target (must be followed by MovingDotsTargetB).
 * Represents a square area with an arbitrary number of white dots 
 * moving with a certain level of coherence. When the newsomeDots parameter
 * is 1 the dots will behave as in Newsome and Pare (1988), when it is 0
 * they will move in a random walk.
 * 
 */
class MovingDotsTargetA : public Target {
public:
	MovingDotsTargetA();
	MovingDotsTargetA(double centerX, double centerY, double width, double coherence);
	void copyToOutputs(real_T *u, int offset);
	bool cursorInTarget(double x, double y);
	bool cursorInTarget(Point p);

	/** The x coordinate of the center of the square target */
	double centerX;

	/** The y coordinate of the center of the square target */
	double centerY;

	/** The width (and height) of the square) */
	double width;

	/** The level of coherence (from 0 to 100). */
    double coherence;
};

/**
 * Default constructor sets all values to zero.
 */
MovingDotsTargetA::MovingDotsTargetA() {
	this->centerX = 0.0;
	this->centerY = 0.0;
	this->width = 0.0;
	this->coherence = 0.0;
}

/**
 * Creates a MovingDotsTargetA with the requested position, size, and coherence.
 * @param centerX the x coordinate of the center of the target.
 * @param centerY the y coordinate of the center of the target.
 * @param width the width and height of the square target.
 * @param coherence the level of coherence of the moving dots.
 */
MovingDotsTargetA::MovingDotsTargetA(double centerX, double centerY, double width, double coherence) {
	this->centerX = centerX;
	this->centerY = centerY;
	this->width = width;
	this->coherence = coherence;
}

/**
 * Determines if the specified point within the target.
 * This function will return whether the specified point is
 * within the target.
 * @param x the x coordinate of the point.
 * @param y the y coordinate of the point.
 * @return true if the specified point is within the bounds of 
 * of the target, false otherwise.
 */
bool MovingDotsTargetA::cursorInTarget(double x, double y) {
	return ( (x > centerX - width/2) && (x < centerX + width/2) && 
		(y > centerY - width/2) && (y < centerY + width/2) );
}

/**
 * Determines if the specified point within the target.
 * Determines if the specified point within the target.
 * This function will return whether the specified point is
 * within the target.
 * @param p a Point containg the x,y coordinate to check
 * @return true if the specified point is within the bounds of 
 * of the target, false otherwise.
 */
bool MovingDotsTargetA::cursorInTarget(Point p) {
	return this->cursorInTarget(p.x, p.y);
}

/**
 * Copies the target to the outputs array.
 * This function will the internal 
 * data of the target to the list of five real_Ts that is necessary
 * to send to the graphics program. This function is called from
 * Behavior::writeOutputs and you should never have to call it 
 * directly.
 * @param u the output buffer to write to.
 * @param offset the location to begin writing.
 */
void MovingDotsTargetA::copyToOutputs(real_T *u, int offset) {
	u[0+offset] = (real_T)(TARGET_TYPE_MOVING_DOTS);
	u[1+offset] = (real_T)(centerX - width / 2);
	u[2+offset] = (real_T)(centerY + width / 2);
	u[3+offset] = (real_T)(centerX + width / 2);
	u[4+offset] = (real_T)(coherence);
}


/***************************************************
 * MovingDotsTargetB
 ***************************************************/

/**
 * Second part of a moving dots target (must be preceded by MovingDotsTargetA).
 * Represents a square area with an arbitrary number of white dots 
 * moving with a certain level of coherence. When the newsomeDots parameter
 * is 1 the dots will behave as in Newsome and Pare (1988), when it is 0
 * they will move in a random walk.
 * 
 */
class MovingDotsTargetB : public Target {
public:
	MovingDotsTargetB();
	MovingDotsTargetB(double direction, double speed, int num_dots, double dot_radius, int newsome_dots);
    bool cursorInTarget(double x, double y);
	bool cursorInTarget(Point p);
	void copyToOutputs(real_T *u, int offset);

	/** The direction of movement of the dots */
	double direction;

	/** The speed of the dots */
	double speed;

	/** The number of dots */
	int num_dots;

	/** The radius of each dot */
    double dot_radius;

	/** The type of movement */
	int newsome_dots;
};

/**
 * Default constructor sets all values to zero.
 */
MovingDotsTargetB::MovingDotsTargetB() {
	this->direction = 0.0;
	this->speed = 0.0;
	this->num_dots = 0;
	this->dot_radius = 0.0;
	this->newsome_dots = 0;
}

/**
 * Creates a MovingDotsTargetB with the requested parameters.
 * @param direction the direction of movement of the dots.
 * @param speed the speed of movement of the dots.
 * @param num_dots the number of dots displayed.
 * @param dot_radius the radius of each dot.
 * @param newsome_dots the type of movement of the dots.
 */
MovingDotsTargetB::MovingDotsTargetB(double direction, double speed, int num_dots, double dot_radius, int newsome_dots) {
	this->direction = direction;
	this->speed = speed;
	this->num_dots = num_dots;
	this->dot_radius = dot_radius;
	this->newsome_dots = newsome_dots;
}

bool MovingDotsTargetB::cursorInTarget(double x, double y) {
	return 0;
}

bool MovingDotsTargetB::cursorInTarget(Point p) {
	return 0;
}

/**
 * Copies the target to the outputs array.
 * This function will the internal 
 * data of the target to the list of five real_Ts that is necessary
 * to send to the graphics program. This function is called from
 * Behavior::writeOutputs and you should never have to call it 
 * directly.
 * @param u the output buffer to write to.
 * @param offset the location to begin writing.
 */
void MovingDotsTargetB::copyToOutputs(real_T *u, int offset) {
	u[0+offset] = (real_T)(direction);
	u[1+offset] = (real_T)(speed);
	u[2+offset] = (real_T)(num_dots);
	u[3+offset] = (real_T)(dot_radius);
	u[4+offset] = (real_T)(newsome_dots);
}

/***************************************************
 * VoltageTarget
 ***************************************************/

/**
 * Voltage target. 3D reach task.
 * Represents a range in voltage staircase for given proximity sensor.
 */
class VoltageTarget : public Target {
public:
    VoltageTarget();
    VoltageTarget(int trow, int tcol);
    void copyToOutputs(real_T *u, int offset);
    bool voltageInTarget(float targetStaircase);
    
    int trow;
    int tcol;
//     float targetVoltageLow;
//     float targetVoltageHigh;
}

/**
 * Default constructor sets all values to zero.
 */
VoltageTarget::VoltageTarget() {
    this->trow = 0;
    this->tcol = 0;
}

VoltageTarget::VoltageTarget(int trow, int tcol) {
    this->trow = trow;
    this->tcol = tcol;
}
/**
 * Determines if the specified input voltage is within the voltage range.
 * This function will return whether the input voltage is
 * within the target voltage range.
 * @param targetStaircase is the input voltage.
 * @param targetVoltageLow is the lower voltage bound for target.
 * @param targetVoltageHigh is the upper voltage bound for target.
 * @return true if the specified input voltage is within the voltage bounds of
 * of the target, false otherwise.
 */
bool VoltageTarget::voltageInTarget(float targetStaircase) {
    
    double targetVoltageLow;
    double targetVoltageHigh;
    
    if (this->trow == 1 && this->tcol == 1){
        targetVoltageLow = 0.4;
        targetVoltageHigh = 0.8;
    } else if (this->trow == 1 && this->tcol == 2){
        targetVoltageLow = 1;
        targetVoltageHigh = 1.5;
    } else if (this->trow == 1 && this->tcol == 3){
        targetVoltageLow = 1.7;
        targetVoltageHigh = 2.1;
    } else if (this->trow == 2 && this->tcol == 1){
        targetVoltageLow = 2.3;
        targetVoltageHigh = 2.7;
    } else if (this->trow == 2 && this->tcol == 3){
        targetVoltageLow = 3;
        targetVoltageHigh = 3.4;
    } else if (this->trow == 3 && this->tcol == 1){
        targetVoltageLow = 3.6;
        targetVoltageHigh = 4;
    } else if (this->trow == 3 && this->tcol == 2){
        targetVoltageLow = 4.2;
        targetVoltageHigh = 4.6;
    } else if (this->trow == 3 && this->tcol == 3){
        targetVoltageLow = 4.8;
        targetVoltageHigh = 5.2;
    } else {
        targetVoltageLow = -11;
        targetVoltageHigh = -10;
    };
    
    return ((targetStaircase > targetVoltageLow) && (targetStaircase < targetVoltageHigh));
};



