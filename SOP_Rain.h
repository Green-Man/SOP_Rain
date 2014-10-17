class SOP_Rain : public SOP_Node
{
public:
    SOP_Rain( OP_Network* net, const char* name, OP_Operator* op);
    virtual ~SOP_Rain();
    static PRM_Template    myTemplateList[];
    static OP_Node* myConstructor(OP_Network*, const char *, OP_Operator *);
protected:
    virtual OP_ERROR     cookMySop(OP_Context &context);
private:
    static bool isParameterChanged_;
    static bool isPointsNumberChanged_;
    static bool isPointsGenerated_;
    static int parmChanged(void *,int,float,const PRM_Template *);
    static int pointsNumberChanged(void *,int,float,const PRM_Template *);
    void generatePoints(GU_Detail* gdp, long n);

    fpreal  SPEED(float t)   { return evalFloat( 0, 0, t); }
    fpreal  FPS(float t)   { return evalFloat( 1, 0, t); }
    UT_Vector3 BOUNDMIN(float t)
            {
                static UT_Vector3 vals;
                for(int i = 0; i < 3; i++)
                    vals[i] = evalFloat( 2, i, t);
                return vals; 
            }
    UT_Vector3 BOUNDMAX(fpreal t)                     //TODO: change to eval all
            {                                      // parm by name
                static UT_Vector3 vals;
                for(int i = 0; i < 3; i++)
                    vals[i] = evalFloat( 3, i, t);
                return vals; 
            }         

    long    NPOINTS(float t)   { return evalInt( 4, 0, t); }
    int     SEED(float t)   { return evalInt( 5, 0, t); }
    fpreal  DICEMIN(float t)   { return evalFloat( 6, 0, t); }
    fpreal  DICEMAX(float t)   { return evalFloat( 7, 0, t); }
    UT_Vector3 RAINDIRECTION(fpreal t)
            {
                static UT_Vector3 vals;
                for(int i = 0; i < 3; i++)
                    vals[i] = evalFloat( 8, i, t);
                return vals; 
            }  
    fpreal  SPEEDVARIENCE(float t)   { return evalFloat( 9, 0, t); }
    fpreal  TIME(float t)   { return evalFloat( 10, 0, t); }


};