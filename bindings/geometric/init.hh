#include <nanobind/nanobind.h>

namespace ompl::binding::geometric
{
    void initPlannersCforest_CForest(nanobind::module_& m);
    void initPlannersCforest_CForestStateSampler(nanobind::module_& m);
    void initPlannersCforest_CForestStateSpaceWrapper(nanobind::module_& m);
    void initPlannersEst_BiEST(nanobind::module_& m);
    void initPlannersEst_EST(nanobind::module_& m);
    void initPlannersEst_ProjEST(nanobind::module_& m);
    void initPlannersExperience_LightningRetrieveRepair(nanobind::module_& m);
    void initPlannersExperience_ThunderRetrieveRepair(nanobind::module_& m);
    void initPlannersFmt_BFMT(nanobind::module_& m);
    void initPlannersFmt_FMT(nanobind::module_& m);
    void initPlannersInformedtreesAitstar_Edge(nanobind::module_& m);
    void initPlannersInformedtreesAitstar_ImplicitGraph(nanobind::module_& m);
    void initPlannersInformedtreesAitstar_Queuetypes(nanobind::module_& m);
    void initPlannersInformedtreesAitstar_Vertex(nanobind::module_& m);
    void initPlannersInformedtreesBitstar_CostHelper(nanobind::module_& m);
    // void initPlannersInformedtreesBitstar_HelperFunctions(nanobind::module_& m);
    void initPlannersInformedtreesBitstar_IdGenerator(nanobind::module_& m);
    void initPlannersInformedtreesBitstar_ImplicitGraph(nanobind::module_& m);
    void initPlannersInformedtreesBitstar_SearchQueue(nanobind::module_& m);
    void initPlannersInformedtreesBitstar_Vertex(nanobind::module_& m);
    void initPlannersInformedtreesEitstar_Direction(nanobind::module_& m);
    void initPlannersInformedtreesEitstar_Edge(nanobind::module_& m);
    void initPlannersInformedtreesEitstar_ForwardQueue(nanobind::module_& m);
    void initPlannersInformedtreesEitstar_RandomGeometricGraph(nanobind::module_& m);
    void initPlannersInformedtreesEitstar_ReverseQueue(nanobind::module_& m);
    void initPlannersInformedtreesEitstar_State(nanobind::module_& m);
    void initPlannersInformedtreesEitstar_Vertex(nanobind::module_& m);
    void initPlannersInformedtrees_ABITstar(nanobind::module_& m);
    void initPlannersInformedtrees_AITstar(nanobind::module_& m);
    void initPlannersInformedtrees_BITstar(nanobind::module_& m);
    void initPlannersInformedtrees_EIRMstar(nanobind::module_& m);
    void initPlannersInformedtrees_EITstar(nanobind::module_& m);
    void initPlannersKpiece_BKPIECE1(nanobind::module_& m);
    void initPlannersKpiece_Discretization(nanobind::module_& m);
    void initPlannersKpiece_KPIECE1(nanobind::module_& m);
    void initPlannersKpiece_LBKPIECE1(nanobind::module_& m);
    void initPlannersPdst_PDST(nanobind::module_& m);
    void initPlannersPrm_ConnectionStrategy(nanobind::module_& m);
    void initPlannersPrm_LazyPRM(nanobind::module_& m);
    void initPlannersPrm_LazyPRMstar(nanobind::module_& m);
    void initPlannersPrm_PRM(nanobind::module_& m);
    void initPlannersPrm_PRMstar(nanobind::module_& m);
    void initPlannersPrm_SPARS(nanobind::module_& m);
    void initPlannersPrm_SPARStwo(nanobind::module_& m);
    void initPlannersRlrt_BiRLRT(nanobind::module_& m);
    void initPlannersRlrt_RLRT(nanobind::module_& m);
    void initPlannersRrt_BiTRRT(nanobind::module_& m);
    void initPlannersRrt_InformedRRTstar(nanobind::module_& m);
    void initPlannersRrt_LBTRRT(nanobind::module_& m);
    void initPlannersRrt_LazyLBTRRT(nanobind::module_& m);
    void initPlannersRrt_LazyRRT(nanobind::module_& m);
    void initPlannersRrt_RRT(nanobind::module_& m);
    void initPlannersRrt_RRTConnect(nanobind::module_& m);
    void initPlannersRrt_RRTXstatic(nanobind::module_& m);
    void initPlannersRrt_RRTsharp(nanobind::module_& m);
    void initPlannersRrt_RRTstar(nanobind::module_& m);
    void initPlannersRrt_SORRTstar(nanobind::module_& m);
    void initPlannersRrt_STRRTstar(nanobind::module_& m);
    void initPlannersRrt_TRRT(nanobind::module_& m);
    void initPlannersRrt_TSRRT(nanobind::module_& m);
    void initPlannersRrt_VFRRT(nanobind::module_& m);
    void initPlannersRrt_pRRT(nanobind::module_& m);
    void initPlannersSbl_SBL(nanobind::module_& m);
    void initPlannersSbl_pSBL(nanobind::module_& m);
    void initPlannersSst_SST(nanobind::module_& m);
    void initPlannersStride_STRIDE(nanobind::module_& m);
    void initPlannersXxl_XXL(nanobind::module_& m);
    void initPlannersXxl_XXLDecomposition(nanobind::module_& m);
    void initPlannersXxl_XXLPlanarDecomposition(nanobind::module_& m);
    void initPlannersXxl_XXLPositionDecomposition(nanobind::module_& m);
    void initPlanners_AnytimePathShortening(nanobind::module_& m);
    void initPlanners_PlannerIncludes(nanobind::module_& m);
    void init_GeneticSearch(nanobind::module_& m);
    void init_HillClimbing(nanobind::module_& m);
    void init_PathGeometric(nanobind::module_& m);
    void init_PathHybridization(nanobind::module_& m);
    void init_PathSimplifier(nanobind::module_& m);
    void init_SimpleSetup(nanobind::module_& m);
}  // namespace ompl::binding::geometric
