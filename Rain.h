class RainData
{
friend class SOP_Rain;
friend class PerformParallelCalculations;

public:
    RainData();
    RainData(   long n, fpreal* bmin, fpreal* bmax, UT_Matrix3 dir,
                fpreal rndMin, fpreal rndMax, int seed );
    ~RainData();

    // THREADED_METHOD1(   RainData, pointsNumber_ > 1,
    //                     computeInitialPositions,
    //                     UT_Vector3 *, 
    //                     pointInitialPositions_)
    // void computeInitialPositionsPartial(UT_Vector3* pointInitialPositions_,
    //                                     const UT_JobInfo &info);
    
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
    const UT_Vector3* getInboundPoints( UT_Vector3* points );

protected:

    
private:
    long pointsNumber_;
    UT_Matrix3 rainDirection_;
    fpreal* minimumBounds_;
    fpreal* maximumBounds_;
    UT_PNoise noise_;
    fpreal rndMin_;
    fpreal rndMax_;
    int seed_;
    Imath::Rand32 rndGenerator_;
    static UT_Vector3* pointInitialPositions_;

    static bool isInitialPositionCached_;
    static bool isInitialPositionAllocated_;

};