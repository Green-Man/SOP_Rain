#include <stdlib.h>

//GIT

#include <UT/UT_DSOVersion.h>
#include <GU/GU_Detail.h>
#include <PRM/PRM_Include.h>
#include <OP/OP_Director.h>
#include <OP/OP_OperatorTable.h>
#include <SOP/SOP_Node.h>
#include <UT/UT_PNoise.h>
#include <GU/GU_PrimPart.h>
#include <OpenEXR/ImathRandom.h>
#include <tbb/tbb.h>


#include "SOP_Rain.h"
#include "Rain.h"

#include <time.h>
//#define DEBUG

#ifdef DEBUG
# define DEBUG_PRINT(x) printf x
#else
# define DEBUG_PRINT(x) do {} while (0)
#endif


//##############################################################################
//######################### Parameters #########################################
void newSopOperator(OP_OperatorTable *table)
{
     table->addOperator(new OP_Operator("fastrain", "Fast Rain",
                                        SOP_Rain::myConstructor,
                                        SOP_Rain::myTemplateList,
                                        0, 0, 0));
}

static PRM_Range        rangeSpeed(PRM_RANGE_UI, 0.0, PRM_RANGE_UI, 10.0);
static PRM_Default      defaultSpeed(3.0);
static PRM_Name         nameSpeed("speed","Speed");

static PRM_Range        rangeFps(PRM_RANGE_UI, 0.5, PRM_RANGE_UI, 60.0);
static PRM_Default      defaultFps(24.0);
static PRM_Name         nameFps("fps", "Fps");

static PRM_Name         nameBoundMin("bmin","BB Min");
static PRM_Default      defaultBoundMin[] = 
                        {
                            PRM_Default(-5.0),
                            PRM_Default(0.0),
                            PRM_Default(-5.0)
                        };

static PRM_Name         nameBoundMax("bmax","BB Max");
static PRM_Default      defaultBoundMax[] = 
                        {
                            PRM_Default(5.0),
                            PRM_Default(10.0),
                            PRM_Default(5.0)
                        };
static PRM_Range        rangeNpoints(PRM_RANGE_UI, 0, PRM_RANGE_UI, 10e+6);
static PRM_Default      defaultNpoints(100);
static PRM_Name         nameNpoints("npoints", "Number Of Points");

static PRM_Range        rangeSeed(PRM_RANGE_UI, 0, PRM_RANGE_UI, 100);
static PRM_Default      defaultSeed(0);
static PRM_Name         nameSeed("seed", "Seed");

static PRM_Range        rangeDiceMin(PRM_RANGE_UI, 0.3, PRM_RANGE_UI, 0.7);
static PRM_Default      defaultDiceMin(0.5);
static PRM_Name         nameDiceMin("diceMin","Dice Min");

static PRM_Range        rangeDiceMax(PRM_RANGE_UI, 0.3, PRM_RANGE_UI, 0.7);
static PRM_Default      defaultDiceMax(0.6);
static PRM_Name         nameDiceMax("diceMax", "Dice Max");

static PRM_Name         nameRainDirection("dir", "Rain Dir");
static PRM_Default      defaultRainDirection[] = 
                        {
                            PRM_Default(0.0),
                            PRM_Default(-1.0),
                            PRM_Default(0.0)
                        };

static PRM_Range        rangeSpeedVarience(PRM_RANGE_UI,0.0,PRM_RANGE_UI,1.0);
static PRM_Default      defaultSpeedVarience(0.2);
static PRM_Name         nameSpeedVarience("speedVarience","Speed Varience");

static PRM_Range        rangeTime(PRM_RANGE_UI,0.0,PRM_RANGE_UI,1.0);
static PRM_Default      defaultTime(0, "hou.time()", CH_PYTHON_EXPRESSION);
static PRM_Name         nameTime("time","Time");                       



PRM_Template SOP_Rain::myTemplateList[] = {
    PRM_Template(   PRM_FLT, 1, &nameSpeed, &defaultSpeed, 0, &rangeSpeed),
    PRM_Template(   PRM_FLT_E, 1, &nameFps, &defaultFps, 0, &rangeFps ),
    PRM_Template(   PRM_XYZ, 3, &nameBoundMin, defaultBoundMin,0,0,parmChanged),
    PRM_Template(   PRM_XYZ, 3, &nameBoundMax, defaultBoundMax,0,0,parmChanged),
    PRM_Template(   PRM_INT_LOG_E, 1, &nameNpoints, &defaultNpoints, 0,
                    &rangeNpoints, pointsNumberChanged ),
    PRM_Template(   PRM_INT, 1, &nameSeed, &defaultSeed, 0, &rangeSeed,
                    parmChanged),
    PRM_Template(   PRM_FLT_E, 1, &nameDiceMin, &defaultDiceMin, 0,
                    &rangeDiceMin, parmChanged ),
    PRM_Template(   PRM_FLT_E, 1, &nameDiceMax, &defaultDiceMax, 0, 
                    &rangeDiceMax, parmChanged ),
    PRM_Template(   PRM_DIRECTION, 3, &nameRainDirection, defaultRainDirection,
                    0, 0, parmChanged),
    PRM_Template(   PRM_FLT, 1, &nameSpeedVarience, &defaultSpeedVarience,
                    0, &rangeSpeedVarience ),
    PRM_Template(   PRM_FLT, 1, &nameTime, &defaultTime, 0, &rangeTime),
    PRM_Template(),
};
//##############################################################################
//####################### End of Parameters ####################################


bool SOP_Rain::isParameterChanged_ = true;
bool SOP_Rain::isPointsNumberChanged_ = true;
int SOP_Rain::parmChanged(void * data, int,  float, const PRM_Template *)
{
  isParameterChanged_ = true;
  return 1;
 }
 int SOP_Rain::pointsNumberChanged( void * data, int,  float,
                                    const PRM_Template *)
{
  isPointsNumberChanged_ = true;
  return 1;
 }

OP_Node* SOP_Rain::myConstructor(   OP_Network *net, const char *name,
                                    OP_Operator *op)
{
    return new SOP_Rain(net, name, op);
    printf("SOP_Rain myconstructor...\n");
}

SOP_Rain::SOP_Rain(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
}

SOP_Rain::~SOP_Rain()
{
    RainData rain;
    rain.makeFree();
}

//==== Iinitialization of RainData static members ====
bool RainData::isInitialPositionAllocated_ = false;
bool RainData::isInitialPositionCached_ = false;
UT_Vector3* RainData::pointInitialPositions_;
//====================================================

RainData::RainData()
{
    printf("RainData default constructor\n");
}

RainData::RainData( long n, fpreal* bmin, fpreal* bmax, UT_Matrix3 dir,
                    fpreal rndMin, fpreal rndMax, int seed)
{
    pointsNumber_ = n;
    minimumBounds_ = bmin;
    maximumBounds_ = bmax;
    rainDirection_ = dir;
    rndMin_ = rndMin;
    rndMax_ = rndMax;
    seed_ = seed;
    noise_.initNoise();


    Imath::Rand32 tmpGenerator( seed_ );
    rndGenerator_ = tmpGenerator;
    //printf("RainData constructor\n");
}

RainData::~RainData()
{
    //printf("RainData destructor\n");
}


class PerformParallelCalculations
{
private:
    RainData* pRain_;

public:
    void operator() (const tbb::blocked_range<long>& r) const
    {   
        bool placed;
        fpreal noiseValue;
        fpreal dice = 1.0;

        UT_Vector3  p, pSeed;

        for(long i=r.begin(); i != r.end(); ++i)
        {
            placed = false;
            dice =  (pRain_->rndMax_ - pRain_->rndMin_) * 
                    (float)pRain_->rndGenerator_.nexti() / 0xffffffff + 
                    pRain_->rndMin_;
            //TODO: try UT_fastRandom(void)
            while (placed == false)     //TODO: Limit the number of tries
            {   
                //p = UT_Vector3(0,2,0);
                p[0] =  (float)pRain_->rndGenerator_.nexti() / 0xffffffff * 
                        (pRain_->maximumBounds_[0]-pRain_->minimumBounds_[0]) + 
                        pRain_->minimumBounds_[0];
                p[1] =  (float)pRain_->rndGenerator_.nexti() / 0xffffffff * 
                        (pRain_->maximumBounds_[1]-pRain_->minimumBounds_[1]) + 
                        pRain_->minimumBounds_[1];
                p[2] =  (float)pRain_->rndGenerator_.nexti() / 0xffffffff * 
                        (pRain_->maximumBounds_[2]-pRain_->minimumBounds_[2]) + 
                        pRain_->minimumBounds_[2];

                pSeed = rowVecMult( p, pRain_->rainDirection_ );
                pSeed[1] = 0;

                float pTmp[3];  //TODO: Noise frequence and offset parms
                uint period[3] = {100,100,100};
                for (int k = 0; k < 3; ++k)
                {
                    pTmp[k] = pSeed[k];
                }
                noiseValue =    ( pRain_->noise_.noise3D(pTmp, period) );
                
                if (dice <= noiseValue)  
                    placed = true;
            };
            pRain_->pointInitialPositions_[i] = p;
        }
    }

    PerformParallelCalculations(RainData* rain) :
        pRain_(rain)
    {};

};



void RainData::computeInitialPositions()
{
    printf( "...Generating Initial Position for %li...\n", pointsNumber_ );
    tbb::parallel_for(  tbb::blocked_range<long>( 0, pointsNumber_ ),
                        PerformParallelCalculations(this) );
    // for (int i = 0; i < pointsNumber_; ++i)
    // {
    //     pointInitialPositions_[i] = UT_Vector3(0,0,0);
    // }

}

UT_Matrix3 SOP_Rain::computeRotationMatrix(UT_Vector3 rainDirection)
{
    static const UT_Vector3 up(0.0, -1.0, 0.0);
    UT_Matrix3 rotDirMatrix(1.0);
    rainDirection.normalize();
    UT_Vector3 axis = cross (up, rainDirection);
    axis.normalize();
    fpreal angle = acos (dot (up, rainDirection));
    rotDirMatrix.rotate (axis, -angle); // minus for proper generation
                                        // of the noise in
                                        // RainData::computeInitialPositions
    return rotDirMatrix;
}

const UT_Vector3* RainData::getInboundPoints( UT_Vector3* points)
{   
    static UT_Vector3 pointPair[2];  
    bool inBound; 
    int endsFound = 0;
    int i = 0;
    while(i<6 && endsFound <2)
    {   
        inBound = false;        
        inBound =   (points[i][0] >= minimumBounds_[0]) &&
                    (points[i][1] >= minimumBounds_[1]) &&
                    (points[i][2] >= minimumBounds_[2]) &&
                    (points[i][0] <= maximumBounds_[0]) &&
                    (points[i][1] <= maximumBounds_[1]) &&
                    (points[i][2] <= maximumBounds_[2]);
        if (inBound == true)
        {
            pointPair[endsFound] = points[i];
            endsFound++;
        }         
        i++;
    }
    if (pointPair[0][1]<pointPair[1][1])
    {
        UT_Vector3 tmpPoint = pointPair[1];
        pointPair[1] = pointPair[0];
        pointPair[0] = tmpPoint;
    }
    return pointPair;
}

OP_ERROR
SOP_Rain::cookMySop(OP_Context &context)
{
    UT_Interrupt    *boss;
    if (error() < UT_ERROR_ABORT)
    {
        boss = UTgetInterrupt();
        boss->opStart("Start generating rain");

        gdp->clearAndDestroy();

        fpreal now = TIME(context.getTime());
        long nPoints = NPOINTS(now);
        int seed = SEED(now);
        fpreal constantSpeed = SPEED (now);
        fpreal speedVarience = SPEEDVARIENCE (now);
        fpreal *bmax, *bmin;
        bmax = BOUNDMAX (now);
        bmin = BOUNDMIN (now);
        UT_Vector3 rainDirection = RAINDIRECTION(now);
        rainDirection.normalize();
            
            //CLOCK
        // clock_t start, end;
        // start = clock();
        RainData rain(  nPoints, bmin, bmax,
                        computeRotationMatrix(rainDirection), 
                        DICEMIN(now), DICEMAX(now), seed);


        if(rain.getAllocationState() == false || isPointsNumberChanged_ == true)
        {
            rain.allocate(NPOINTS(now));
        }
        if( rain.getAllocationState() == true && 
            ( rain.getCachedState() == false || isParameterChanged_ == true ) )
        {   
            rain.computeInitialPositions();
            rain.setCachedState(true);      
        }
        
        // end = clock();
        // printf("positioning: %f\n", (double)(end-start)/CLOCKS_PER_SEC);
            //CLOCK END

        GU_PrimParticle* partsys;
        if (partsys = GU_PrimParticle::build(gdp, nPoints))
        {
            int pointIndex;
            fpreal actualSpeed;
            fpreal parm1x,parm2x,parm1y,parm2y,parm1z,parm2z, parmInitial;
            UT_Vector3 p1x, p2x, p1y, p2y, p1z, p2z, p1, p2, initialPosition, p;
            fpreal pathLength, pathPeriod;
            fpreal integralPart;
            const UT_Vector3* boundPoints;
            Imath::Rand32 randomGenerator(seed);

            GA_Primitive::const_iterator it;
            
            // //CLOCK
            // start = clock();
            for(partsys->beginVertex(it); it.atEnd() == 0; ++it)
            {   pointIndex = it.getPointOffset();
                actualSpeed =   constantSpeed +
                                constantSpeed * speedVarience *
                                ((float)randomGenerator.nexti() / 0xffffffff
                                 - 0.5) * 2;

                initialPosition = rain.getInitialPosition(pointIndex);

                parm1x = (bmax[0] - initialPosition[0])/rainDirection[0];
                parm2x = (bmin[0] - initialPosition[0])/rainDirection[0];
                parm1y = (bmax[1] - initialPosition[1])/rainDirection[1];
                parm2y = (bmin[1] - initialPosition[1])/rainDirection[1];
                parm1z = (bmax[2] - initialPosition[2])/rainDirection[2];
                parm2z = (bmin[2] - initialPosition[2])/rainDirection[2];
                p1x = initialPosition + parm1x*rainDirection;
                p2x = initialPosition + parm2x*rainDirection; 
                p1y = initialPosition + parm1y*rainDirection;
                p2y = initialPosition + parm2y*rainDirection;
                p1z = initialPosition + parm1z*rainDirection;
                p2z = initialPosition + parm2z*rainDirection;

                UT_Vector3 endsArray [6] = {p1x, p2x, p1y, p2y, p1z, p2z};
                boundPoints = rain.getInboundPoints(endsArray);

                pathLength = (boundPoints[0] - boundPoints[1]).length();
                pathPeriod = pathLength / actualSpeed;
                parmInitial =   (initialPosition - boundPoints[0]).length() / 
                                pathLength;
                integralPart = parmInitial + now / pathPeriod;
                
                p = boundPoints[0] +
                    (integralPart - (int)integralPart) * 
                    (boundPoints[1] - boundPoints[0]);

                gdp->setPos3(pointIndex, p);

            }

            // end = clock();
            // printf("For loop: %f\n", (double)(end-start)/CLOCKS_PER_SEC);
            //CLOCK END
        }
        boss->opEnd();
    }
    isParameterChanged_ = false;
    isPointsNumberChanged_ = false;
    unlockInputs();
    return error();
}
