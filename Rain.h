class RainData
{
friend class SOP_Rain;
friend class PerformParallelCalculations;

public:
    RainData();
    RainData(   fpreal now,
                long n, UT_Vector3 bmin, UT_Vector3 bmax, UT_Vector3 dir,
                fpreal rndMin, fpreal rndMax, int seed, fpreal speed,
                fpreal speedVarience);
    ~RainData();
    
    void setCachedState(bool val)
    {
        isInitialPositionCached_ = val;
    };
    void setAllocationState(bool val)
    {
        isInitialPositionAllocated_ = val;
    };

    bool getCachedState()
    {
        return isInitialPositionCached_;
    }
    bool getAllocationState()
    {
        return isInitialPositionAllocated_;
    }
    void allocate(long nPoints)
    {
        pointInitialPositions_ = new UT_Vector3[nPoints];   //TODO: check for 
        setAllocationState( true );                         //proper deletion
        setCachedState( false );
        printf( "pointInitialPositions_ allocated for %li points\n", nPoints );
    }
    void makeFree()
    {
        delete [] pointInitialPositions_;
        setAllocationState( false );
        printf( "pointInitialPositions_ deleted\n" );
    }

    void computeInitialPositions();
    UT_Vector3 getInitialPosition( long i )
    {
        return pointInitialPositions_[ i ];
    }

    void getShiftedPosition(  GU_Detail* gdp,
                                    GU_PrimParticle* particleSystem);
    const UT_Vector3* getInboundPoints( UT_Vector3* points );

private:
    UT_Matrix3 computeRotationMatrix(UT_Vector3 rainDirection);
    long pointsNumber_;
    UT_Vector3 rainDirection_;
    UT_Matrix3 directionMatrix_;
    UT_Vector3 minimumBounds_;
    UT_Vector3 maximumBounds_;
    UT_PNoise noise_;
    fpreal rndMin_;
    fpreal rndMax_;
    int seed_;
    fpreal constantSpeed_, speedVarience_, now_;

    Imath::Rand32 rndGenerator_;
    static UT_Vector3* pointInitialPositions_;

    static bool isInitialPositionCached_;
    static bool isInitialPositionAllocated_;
};


class PerformParallelCalculations
{
private:
    RainData* pRain_;
public:
    void operator() (const tbb::blocked_range<long>& r) const;
    PerformParallelCalculations(RainData* rain);

};