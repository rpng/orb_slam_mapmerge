##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=orb_slam
ConfigurationName      :=Debug
WorkspacePath          := "/home/patrick/.codelite/orb_slam"
ProjectPath            := "/home/patrick/catkin_ws_orb/src/orb_slam"
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=Patrick
Date                   :=08/07/15
CodeLitePath           :="/home/patrick/.codelite"
LinkerName             :=/usr/bin/g++
SharedObjectLinkerName :=/usr/bin/g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.i
DebugSwitch            :=-g 
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E
ObjectsFileList        :="orb_slam.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). 
IncludePCH             := 
RcIncludePath          := 
Libs                   := 
ArLibs                 :=  
LibPath                := $(LibraryPathSwitch). 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := /usr/bin/ar rcu
CXX      := /usr/bin/g++
CC       := /usr/bin/gcc
CXXFLAGS :=  -g -O0 -Wall $(Preprocessors)
CFLAGS   :=  -g -O0 -Wall $(Preprocessors)
ASFLAGS  := 
AS       := /usr/bin/as


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects0=$(IntermediateDirectory)/src_main.cc$(ObjectSuffix) $(IntermediateDirectory)/src_FORB.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_BowVector.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_FeatureVector.cpp$(ObjectSuffix) $(IntermediateDirectory)/src_ScoringObject.cpp$(ObjectSuffix) $(IntermediateDirectory)/threads_LoopClosing.cc$(ObjectSuffix) $(IntermediateDirectory)/threads_MapMerging.cc$(ObjectSuffix) $(IntermediateDirectory)/threads_Tracking.cc$(ObjectSuffix) $(IntermediateDirectory)/threads_LocalMapping.cc$(ObjectSuffix) $(IntermediateDirectory)/publishers_MapPublisher.cc$(ObjectSuffix) \
	$(IntermediateDirectory)/publishers_FramePublisher.cc$(ObjectSuffix) $(IntermediateDirectory)/util_Converter.cc$(ObjectSuffix) $(IntermediateDirectory)/util_ORBmatcher.cc$(ObjectSuffix) $(IntermediateDirectory)/util_Initializer.cc$(ObjectSuffix) $(IntermediateDirectory)/util_Sim3Solver.cc$(ObjectSuffix) $(IntermediateDirectory)/util_PnPsolver.cc$(ObjectSuffix) $(IntermediateDirectory)/util_Optimizer.cc$(ObjectSuffix) $(IntermediateDirectory)/util_ORBextractor.cc$(ObjectSuffix) $(IntermediateDirectory)/types_MapPoint.cc$(ObjectSuffix) $(IntermediateDirectory)/types_Map.cc$(ObjectSuffix) \
	$(IntermediateDirectory)/types_MapDatabase.cc$(ObjectSuffix) $(IntermediateDirectory)/types_KeyFrameDatabase.cc$(ObjectSuffix) $(IntermediateDirectory)/types_Frame.cc$(ObjectSuffix) $(IntermediateDirectory)/types_KeyFrame.cc$(ObjectSuffix) $(IntermediateDirectory)/stuff_timeutil.cpp$(ObjectSuffix) $(IntermediateDirectory)/stuff_command_args.cpp$(ObjectSuffix) $(IntermediateDirectory)/stuff_property.cpp$(ObjectSuffix) $(IntermediateDirectory)/stuff_opengl_primitives.cpp$(ObjectSuffix) $(IntermediateDirectory)/stuff_tictoc.cpp$(ObjectSuffix) $(IntermediateDirectory)/stuff_sampler.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/stuff_filesys_tools.cpp$(ObjectSuffix) $(IntermediateDirectory)/stuff_string_tools.cpp$(ObjectSuffix) $(IntermediateDirectory)/stuff_os_specific.c$(ObjectSuffix) $(IntermediateDirectory)/stuff_sparse_helper.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_matrix_structure.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_sparse_optimizer.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_hyper_graph_action.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_hyper_graph.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_cache.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_optimization_algorithm_factory.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/core_optimization_algorithm.cpp$(ObjectSuffix) 

Objects1=$(IntermediateDirectory)/core_optimization_algorithm_with_hessian.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_robust_kernel.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_estimate_propagator.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_sparse_optimizer_terminate_action.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_solver.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_factory.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_sparse_block_matrix_test.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_optimization_algorithm_levenberg.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_marginal_covariance_cholesky.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/core_robust_kernel_factory.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_parameter.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_optimizable_graph.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_jacobian_workspace.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_optimization_algorithm_dogleg.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_hyper_dijkstra.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_parameter_container.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_robust_kernel_impl.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_optimization_algorithm_gauss_newton.cpp$(ObjectSuffix) $(IntermediateDirectory)/core_batch_stats.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/dutils_Random.cpp$(ObjectSuffix) $(IntermediateDirectory)/dutils_Timestamp.cpp$(ObjectSuffix) $(IntermediateDirectory)/cholmod_solver_cholmod.cpp$(ObjectSuffix) $(IntermediateDirectory)/dense_solver_dense.cpp$(ObjectSuffix) $(IntermediateDirectory)/sba_types_six_dof_expmap.cpp$(ObjectSuffix) $(IntermediateDirectory)/sba_types_sba.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_edge_se3_pointxyz_disparity.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_parameter_camera.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_isometry3d_mappings.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_edge_se3.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/slam3d_dquat2mat.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_parameter_stereo_camera.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_vertex_se3.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_edge_se3_pointxyz_depth.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_parameter_se3_offset.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_edge_se3_offset.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_edge_se3_prior.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_vertex_pointxyz.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_isometry3d_gradients.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_edge_se3_pointxyz.cpp$(ObjectSuffix) \
	$(IntermediateDirectory)/slam3d_dquat2mat_maxima_generated.cpp$(ObjectSuffix) $(IntermediateDirectory)/slam3d_types_slam3d.cpp$(ObjectSuffix) 

Objects2=$(IntermediateDirectory)/sim3_types_seven_dof_expmap.cpp$(ObjectSuffix) 



Objects=$(Objects0) $(Objects1) $(Objects2) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	@echo $(Objects1) >> $(ObjectsFileList)
	@echo $(Objects2) >> $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

$(IntermediateDirectory)/.d:
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/src_main.cc$(ObjectSuffix): orb_slam/src/main.cc $(IntermediateDirectory)/src_main.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/main.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_main.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_main.cc$(DependSuffix): orb_slam/src/main.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_main.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/src_main.cc$(DependSuffix) -MM "orb_slam/src/main.cc"

$(IntermediateDirectory)/src_main.cc$(PreprocessSuffix): orb_slam/src/main.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_main.cc$(PreprocessSuffix) "orb_slam/src/main.cc"

$(IntermediateDirectory)/src_FORB.cpp$(ObjectSuffix): dbow2/src/FORB.cpp $(IntermediateDirectory)/src_FORB.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/dbow2/src/FORB.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_FORB.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_FORB.cpp$(DependSuffix): dbow2/src/FORB.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_FORB.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_FORB.cpp$(DependSuffix) -MM "dbow2/src/FORB.cpp"

$(IntermediateDirectory)/src_FORB.cpp$(PreprocessSuffix): dbow2/src/FORB.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_FORB.cpp$(PreprocessSuffix) "dbow2/src/FORB.cpp"

$(IntermediateDirectory)/src_BowVector.cpp$(ObjectSuffix): dbow2/src/BowVector.cpp $(IntermediateDirectory)/src_BowVector.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/dbow2/src/BowVector.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_BowVector.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_BowVector.cpp$(DependSuffix): dbow2/src/BowVector.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_BowVector.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_BowVector.cpp$(DependSuffix) -MM "dbow2/src/BowVector.cpp"

$(IntermediateDirectory)/src_BowVector.cpp$(PreprocessSuffix): dbow2/src/BowVector.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_BowVector.cpp$(PreprocessSuffix) "dbow2/src/BowVector.cpp"

$(IntermediateDirectory)/src_FeatureVector.cpp$(ObjectSuffix): dbow2/src/FeatureVector.cpp $(IntermediateDirectory)/src_FeatureVector.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/dbow2/src/FeatureVector.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_FeatureVector.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_FeatureVector.cpp$(DependSuffix): dbow2/src/FeatureVector.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_FeatureVector.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_FeatureVector.cpp$(DependSuffix) -MM "dbow2/src/FeatureVector.cpp"

$(IntermediateDirectory)/src_FeatureVector.cpp$(PreprocessSuffix): dbow2/src/FeatureVector.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_FeatureVector.cpp$(PreprocessSuffix) "dbow2/src/FeatureVector.cpp"

$(IntermediateDirectory)/src_ScoringObject.cpp$(ObjectSuffix): dbow2/src/ScoringObject.cpp $(IntermediateDirectory)/src_ScoringObject.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/dbow2/src/ScoringObject.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/src_ScoringObject.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/src_ScoringObject.cpp$(DependSuffix): dbow2/src/ScoringObject.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/src_ScoringObject.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/src_ScoringObject.cpp$(DependSuffix) -MM "dbow2/src/ScoringObject.cpp"

$(IntermediateDirectory)/src_ScoringObject.cpp$(PreprocessSuffix): dbow2/src/ScoringObject.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/src_ScoringObject.cpp$(PreprocessSuffix) "dbow2/src/ScoringObject.cpp"

$(IntermediateDirectory)/threads_LoopClosing.cc$(ObjectSuffix): orb_slam/src/threads/LoopClosing.cc $(IntermediateDirectory)/threads_LoopClosing.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/threads/LoopClosing.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/threads_LoopClosing.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/threads_LoopClosing.cc$(DependSuffix): orb_slam/src/threads/LoopClosing.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/threads_LoopClosing.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/threads_LoopClosing.cc$(DependSuffix) -MM "orb_slam/src/threads/LoopClosing.cc"

$(IntermediateDirectory)/threads_LoopClosing.cc$(PreprocessSuffix): orb_slam/src/threads/LoopClosing.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/threads_LoopClosing.cc$(PreprocessSuffix) "orb_slam/src/threads/LoopClosing.cc"

$(IntermediateDirectory)/threads_MapMerging.cc$(ObjectSuffix): orb_slam/src/threads/MapMerging.cc $(IntermediateDirectory)/threads_MapMerging.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/threads/MapMerging.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/threads_MapMerging.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/threads_MapMerging.cc$(DependSuffix): orb_slam/src/threads/MapMerging.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/threads_MapMerging.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/threads_MapMerging.cc$(DependSuffix) -MM "orb_slam/src/threads/MapMerging.cc"

$(IntermediateDirectory)/threads_MapMerging.cc$(PreprocessSuffix): orb_slam/src/threads/MapMerging.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/threads_MapMerging.cc$(PreprocessSuffix) "orb_slam/src/threads/MapMerging.cc"

$(IntermediateDirectory)/threads_Tracking.cc$(ObjectSuffix): orb_slam/src/threads/Tracking.cc $(IntermediateDirectory)/threads_Tracking.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/threads/Tracking.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/threads_Tracking.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/threads_Tracking.cc$(DependSuffix): orb_slam/src/threads/Tracking.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/threads_Tracking.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/threads_Tracking.cc$(DependSuffix) -MM "orb_slam/src/threads/Tracking.cc"

$(IntermediateDirectory)/threads_Tracking.cc$(PreprocessSuffix): orb_slam/src/threads/Tracking.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/threads_Tracking.cc$(PreprocessSuffix) "orb_slam/src/threads/Tracking.cc"

$(IntermediateDirectory)/threads_LocalMapping.cc$(ObjectSuffix): orb_slam/src/threads/LocalMapping.cc $(IntermediateDirectory)/threads_LocalMapping.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/threads/LocalMapping.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/threads_LocalMapping.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/threads_LocalMapping.cc$(DependSuffix): orb_slam/src/threads/LocalMapping.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/threads_LocalMapping.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/threads_LocalMapping.cc$(DependSuffix) -MM "orb_slam/src/threads/LocalMapping.cc"

$(IntermediateDirectory)/threads_LocalMapping.cc$(PreprocessSuffix): orb_slam/src/threads/LocalMapping.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/threads_LocalMapping.cc$(PreprocessSuffix) "orb_slam/src/threads/LocalMapping.cc"

$(IntermediateDirectory)/publishers_MapPublisher.cc$(ObjectSuffix): orb_slam/src/publishers/MapPublisher.cc $(IntermediateDirectory)/publishers_MapPublisher.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/publishers/MapPublisher.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/publishers_MapPublisher.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/publishers_MapPublisher.cc$(DependSuffix): orb_slam/src/publishers/MapPublisher.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/publishers_MapPublisher.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/publishers_MapPublisher.cc$(DependSuffix) -MM "orb_slam/src/publishers/MapPublisher.cc"

$(IntermediateDirectory)/publishers_MapPublisher.cc$(PreprocessSuffix): orb_slam/src/publishers/MapPublisher.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/publishers_MapPublisher.cc$(PreprocessSuffix) "orb_slam/src/publishers/MapPublisher.cc"

$(IntermediateDirectory)/publishers_FramePublisher.cc$(ObjectSuffix): orb_slam/src/publishers/FramePublisher.cc $(IntermediateDirectory)/publishers_FramePublisher.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/publishers/FramePublisher.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/publishers_FramePublisher.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/publishers_FramePublisher.cc$(DependSuffix): orb_slam/src/publishers/FramePublisher.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/publishers_FramePublisher.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/publishers_FramePublisher.cc$(DependSuffix) -MM "orb_slam/src/publishers/FramePublisher.cc"

$(IntermediateDirectory)/publishers_FramePublisher.cc$(PreprocessSuffix): orb_slam/src/publishers/FramePublisher.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/publishers_FramePublisher.cc$(PreprocessSuffix) "orb_slam/src/publishers/FramePublisher.cc"

$(IntermediateDirectory)/util_Converter.cc$(ObjectSuffix): orb_slam/src/util/Converter.cc $(IntermediateDirectory)/util_Converter.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/util/Converter.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_Converter.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_Converter.cc$(DependSuffix): orb_slam/src/util/Converter.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_Converter.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/util_Converter.cc$(DependSuffix) -MM "orb_slam/src/util/Converter.cc"

$(IntermediateDirectory)/util_Converter.cc$(PreprocessSuffix): orb_slam/src/util/Converter.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_Converter.cc$(PreprocessSuffix) "orb_slam/src/util/Converter.cc"

$(IntermediateDirectory)/util_ORBmatcher.cc$(ObjectSuffix): orb_slam/src/util/ORBmatcher.cc $(IntermediateDirectory)/util_ORBmatcher.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/util/ORBmatcher.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_ORBmatcher.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_ORBmatcher.cc$(DependSuffix): orb_slam/src/util/ORBmatcher.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_ORBmatcher.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/util_ORBmatcher.cc$(DependSuffix) -MM "orb_slam/src/util/ORBmatcher.cc"

$(IntermediateDirectory)/util_ORBmatcher.cc$(PreprocessSuffix): orb_slam/src/util/ORBmatcher.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_ORBmatcher.cc$(PreprocessSuffix) "orb_slam/src/util/ORBmatcher.cc"

$(IntermediateDirectory)/util_Initializer.cc$(ObjectSuffix): orb_slam/src/util/Initializer.cc $(IntermediateDirectory)/util_Initializer.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/util/Initializer.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_Initializer.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_Initializer.cc$(DependSuffix): orb_slam/src/util/Initializer.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_Initializer.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/util_Initializer.cc$(DependSuffix) -MM "orb_slam/src/util/Initializer.cc"

$(IntermediateDirectory)/util_Initializer.cc$(PreprocessSuffix): orb_slam/src/util/Initializer.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_Initializer.cc$(PreprocessSuffix) "orb_slam/src/util/Initializer.cc"

$(IntermediateDirectory)/util_Sim3Solver.cc$(ObjectSuffix): orb_slam/src/util/Sim3Solver.cc $(IntermediateDirectory)/util_Sim3Solver.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/util/Sim3Solver.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_Sim3Solver.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_Sim3Solver.cc$(DependSuffix): orb_slam/src/util/Sim3Solver.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_Sim3Solver.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/util_Sim3Solver.cc$(DependSuffix) -MM "orb_slam/src/util/Sim3Solver.cc"

$(IntermediateDirectory)/util_Sim3Solver.cc$(PreprocessSuffix): orb_slam/src/util/Sim3Solver.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_Sim3Solver.cc$(PreprocessSuffix) "orb_slam/src/util/Sim3Solver.cc"

$(IntermediateDirectory)/util_PnPsolver.cc$(ObjectSuffix): orb_slam/src/util/PnPsolver.cc $(IntermediateDirectory)/util_PnPsolver.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/util/PnPsolver.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_PnPsolver.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_PnPsolver.cc$(DependSuffix): orb_slam/src/util/PnPsolver.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_PnPsolver.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/util_PnPsolver.cc$(DependSuffix) -MM "orb_slam/src/util/PnPsolver.cc"

$(IntermediateDirectory)/util_PnPsolver.cc$(PreprocessSuffix): orb_slam/src/util/PnPsolver.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_PnPsolver.cc$(PreprocessSuffix) "orb_slam/src/util/PnPsolver.cc"

$(IntermediateDirectory)/util_Optimizer.cc$(ObjectSuffix): orb_slam/src/util/Optimizer.cc $(IntermediateDirectory)/util_Optimizer.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/util/Optimizer.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_Optimizer.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_Optimizer.cc$(DependSuffix): orb_slam/src/util/Optimizer.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_Optimizer.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/util_Optimizer.cc$(DependSuffix) -MM "orb_slam/src/util/Optimizer.cc"

$(IntermediateDirectory)/util_Optimizer.cc$(PreprocessSuffix): orb_slam/src/util/Optimizer.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_Optimizer.cc$(PreprocessSuffix) "orb_slam/src/util/Optimizer.cc"

$(IntermediateDirectory)/util_ORBextractor.cc$(ObjectSuffix): orb_slam/src/util/ORBextractor.cc $(IntermediateDirectory)/util_ORBextractor.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/util/ORBextractor.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/util_ORBextractor.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/util_ORBextractor.cc$(DependSuffix): orb_slam/src/util/ORBextractor.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/util_ORBextractor.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/util_ORBextractor.cc$(DependSuffix) -MM "orb_slam/src/util/ORBextractor.cc"

$(IntermediateDirectory)/util_ORBextractor.cc$(PreprocessSuffix): orb_slam/src/util/ORBextractor.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/util_ORBextractor.cc$(PreprocessSuffix) "orb_slam/src/util/ORBextractor.cc"

$(IntermediateDirectory)/types_MapPoint.cc$(ObjectSuffix): orb_slam/src/types/MapPoint.cc $(IntermediateDirectory)/types_MapPoint.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/types/MapPoint.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/types_MapPoint.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/types_MapPoint.cc$(DependSuffix): orb_slam/src/types/MapPoint.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/types_MapPoint.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/types_MapPoint.cc$(DependSuffix) -MM "orb_slam/src/types/MapPoint.cc"

$(IntermediateDirectory)/types_MapPoint.cc$(PreprocessSuffix): orb_slam/src/types/MapPoint.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/types_MapPoint.cc$(PreprocessSuffix) "orb_slam/src/types/MapPoint.cc"

$(IntermediateDirectory)/types_Map.cc$(ObjectSuffix): orb_slam/src/types/Map.cc $(IntermediateDirectory)/types_Map.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/types/Map.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/types_Map.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/types_Map.cc$(DependSuffix): orb_slam/src/types/Map.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/types_Map.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/types_Map.cc$(DependSuffix) -MM "orb_slam/src/types/Map.cc"

$(IntermediateDirectory)/types_Map.cc$(PreprocessSuffix): orb_slam/src/types/Map.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/types_Map.cc$(PreprocessSuffix) "orb_slam/src/types/Map.cc"

$(IntermediateDirectory)/types_MapDatabase.cc$(ObjectSuffix): orb_slam/src/types/MapDatabase.cc $(IntermediateDirectory)/types_MapDatabase.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/types/MapDatabase.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/types_MapDatabase.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/types_MapDatabase.cc$(DependSuffix): orb_slam/src/types/MapDatabase.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/types_MapDatabase.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/types_MapDatabase.cc$(DependSuffix) -MM "orb_slam/src/types/MapDatabase.cc"

$(IntermediateDirectory)/types_MapDatabase.cc$(PreprocessSuffix): orb_slam/src/types/MapDatabase.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/types_MapDatabase.cc$(PreprocessSuffix) "orb_slam/src/types/MapDatabase.cc"

$(IntermediateDirectory)/types_KeyFrameDatabase.cc$(ObjectSuffix): orb_slam/src/types/KeyFrameDatabase.cc $(IntermediateDirectory)/types_KeyFrameDatabase.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/types/KeyFrameDatabase.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/types_KeyFrameDatabase.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/types_KeyFrameDatabase.cc$(DependSuffix): orb_slam/src/types/KeyFrameDatabase.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/types_KeyFrameDatabase.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/types_KeyFrameDatabase.cc$(DependSuffix) -MM "orb_slam/src/types/KeyFrameDatabase.cc"

$(IntermediateDirectory)/types_KeyFrameDatabase.cc$(PreprocessSuffix): orb_slam/src/types/KeyFrameDatabase.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/types_KeyFrameDatabase.cc$(PreprocessSuffix) "orb_slam/src/types/KeyFrameDatabase.cc"

$(IntermediateDirectory)/types_Frame.cc$(ObjectSuffix): orb_slam/src/types/Frame.cc $(IntermediateDirectory)/types_Frame.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/types/Frame.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/types_Frame.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/types_Frame.cc$(DependSuffix): orb_slam/src/types/Frame.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/types_Frame.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/types_Frame.cc$(DependSuffix) -MM "orb_slam/src/types/Frame.cc"

$(IntermediateDirectory)/types_Frame.cc$(PreprocessSuffix): orb_slam/src/types/Frame.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/types_Frame.cc$(PreprocessSuffix) "orb_slam/src/types/Frame.cc"

$(IntermediateDirectory)/types_KeyFrame.cc$(ObjectSuffix): orb_slam/src/types/KeyFrame.cc $(IntermediateDirectory)/types_KeyFrame.cc$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/orb_slam/src/types/KeyFrame.cc" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/types_KeyFrame.cc$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/types_KeyFrame.cc$(DependSuffix): orb_slam/src/types/KeyFrame.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/types_KeyFrame.cc$(ObjectSuffix) -MF$(IntermediateDirectory)/types_KeyFrame.cc$(DependSuffix) -MM "orb_slam/src/types/KeyFrame.cc"

$(IntermediateDirectory)/types_KeyFrame.cc$(PreprocessSuffix): orb_slam/src/types/KeyFrame.cc
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/types_KeyFrame.cc$(PreprocessSuffix) "orb_slam/src/types/KeyFrame.cc"

$(IntermediateDirectory)/stuff_timeutil.cpp$(ObjectSuffix): g2o/g2o/stuff/timeutil.cpp $(IntermediateDirectory)/stuff_timeutil.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/stuff/timeutil.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stuff_timeutil.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stuff_timeutil.cpp$(DependSuffix): g2o/g2o/stuff/timeutil.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stuff_timeutil.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/stuff_timeutil.cpp$(DependSuffix) -MM "g2o/g2o/stuff/timeutil.cpp"

$(IntermediateDirectory)/stuff_timeutil.cpp$(PreprocessSuffix): g2o/g2o/stuff/timeutil.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stuff_timeutil.cpp$(PreprocessSuffix) "g2o/g2o/stuff/timeutil.cpp"

$(IntermediateDirectory)/stuff_command_args.cpp$(ObjectSuffix): g2o/g2o/stuff/command_args.cpp $(IntermediateDirectory)/stuff_command_args.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/stuff/command_args.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stuff_command_args.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stuff_command_args.cpp$(DependSuffix): g2o/g2o/stuff/command_args.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stuff_command_args.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/stuff_command_args.cpp$(DependSuffix) -MM "g2o/g2o/stuff/command_args.cpp"

$(IntermediateDirectory)/stuff_command_args.cpp$(PreprocessSuffix): g2o/g2o/stuff/command_args.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stuff_command_args.cpp$(PreprocessSuffix) "g2o/g2o/stuff/command_args.cpp"

$(IntermediateDirectory)/stuff_property.cpp$(ObjectSuffix): g2o/g2o/stuff/property.cpp $(IntermediateDirectory)/stuff_property.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/stuff/property.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stuff_property.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stuff_property.cpp$(DependSuffix): g2o/g2o/stuff/property.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stuff_property.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/stuff_property.cpp$(DependSuffix) -MM "g2o/g2o/stuff/property.cpp"

$(IntermediateDirectory)/stuff_property.cpp$(PreprocessSuffix): g2o/g2o/stuff/property.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stuff_property.cpp$(PreprocessSuffix) "g2o/g2o/stuff/property.cpp"

$(IntermediateDirectory)/stuff_opengl_primitives.cpp$(ObjectSuffix): g2o/g2o/stuff/opengl_primitives.cpp $(IntermediateDirectory)/stuff_opengl_primitives.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/stuff/opengl_primitives.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stuff_opengl_primitives.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stuff_opengl_primitives.cpp$(DependSuffix): g2o/g2o/stuff/opengl_primitives.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stuff_opengl_primitives.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/stuff_opengl_primitives.cpp$(DependSuffix) -MM "g2o/g2o/stuff/opengl_primitives.cpp"

$(IntermediateDirectory)/stuff_opengl_primitives.cpp$(PreprocessSuffix): g2o/g2o/stuff/opengl_primitives.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stuff_opengl_primitives.cpp$(PreprocessSuffix) "g2o/g2o/stuff/opengl_primitives.cpp"

$(IntermediateDirectory)/stuff_tictoc.cpp$(ObjectSuffix): g2o/g2o/stuff/tictoc.cpp $(IntermediateDirectory)/stuff_tictoc.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/stuff/tictoc.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stuff_tictoc.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stuff_tictoc.cpp$(DependSuffix): g2o/g2o/stuff/tictoc.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stuff_tictoc.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/stuff_tictoc.cpp$(DependSuffix) -MM "g2o/g2o/stuff/tictoc.cpp"

$(IntermediateDirectory)/stuff_tictoc.cpp$(PreprocessSuffix): g2o/g2o/stuff/tictoc.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stuff_tictoc.cpp$(PreprocessSuffix) "g2o/g2o/stuff/tictoc.cpp"

$(IntermediateDirectory)/stuff_sampler.cpp$(ObjectSuffix): g2o/g2o/stuff/sampler.cpp $(IntermediateDirectory)/stuff_sampler.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/stuff/sampler.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stuff_sampler.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stuff_sampler.cpp$(DependSuffix): g2o/g2o/stuff/sampler.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stuff_sampler.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/stuff_sampler.cpp$(DependSuffix) -MM "g2o/g2o/stuff/sampler.cpp"

$(IntermediateDirectory)/stuff_sampler.cpp$(PreprocessSuffix): g2o/g2o/stuff/sampler.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stuff_sampler.cpp$(PreprocessSuffix) "g2o/g2o/stuff/sampler.cpp"

$(IntermediateDirectory)/stuff_filesys_tools.cpp$(ObjectSuffix): g2o/g2o/stuff/filesys_tools.cpp $(IntermediateDirectory)/stuff_filesys_tools.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/stuff/filesys_tools.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stuff_filesys_tools.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stuff_filesys_tools.cpp$(DependSuffix): g2o/g2o/stuff/filesys_tools.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stuff_filesys_tools.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/stuff_filesys_tools.cpp$(DependSuffix) -MM "g2o/g2o/stuff/filesys_tools.cpp"

$(IntermediateDirectory)/stuff_filesys_tools.cpp$(PreprocessSuffix): g2o/g2o/stuff/filesys_tools.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stuff_filesys_tools.cpp$(PreprocessSuffix) "g2o/g2o/stuff/filesys_tools.cpp"

$(IntermediateDirectory)/stuff_string_tools.cpp$(ObjectSuffix): g2o/g2o/stuff/string_tools.cpp $(IntermediateDirectory)/stuff_string_tools.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/stuff/string_tools.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stuff_string_tools.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stuff_string_tools.cpp$(DependSuffix): g2o/g2o/stuff/string_tools.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stuff_string_tools.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/stuff_string_tools.cpp$(DependSuffix) -MM "g2o/g2o/stuff/string_tools.cpp"

$(IntermediateDirectory)/stuff_string_tools.cpp$(PreprocessSuffix): g2o/g2o/stuff/string_tools.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stuff_string_tools.cpp$(PreprocessSuffix) "g2o/g2o/stuff/string_tools.cpp"

$(IntermediateDirectory)/stuff_os_specific.c$(ObjectSuffix): g2o/g2o/stuff/os_specific.c $(IntermediateDirectory)/stuff_os_specific.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/stuff/os_specific.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stuff_os_specific.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stuff_os_specific.c$(DependSuffix): g2o/g2o/stuff/os_specific.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stuff_os_specific.c$(ObjectSuffix) -MF$(IntermediateDirectory)/stuff_os_specific.c$(DependSuffix) -MM "g2o/g2o/stuff/os_specific.c"

$(IntermediateDirectory)/stuff_os_specific.c$(PreprocessSuffix): g2o/g2o/stuff/os_specific.c
	@$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stuff_os_specific.c$(PreprocessSuffix) "g2o/g2o/stuff/os_specific.c"

$(IntermediateDirectory)/stuff_sparse_helper.cpp$(ObjectSuffix): g2o/g2o/stuff/sparse_helper.cpp $(IntermediateDirectory)/stuff_sparse_helper.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/stuff/sparse_helper.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/stuff_sparse_helper.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/stuff_sparse_helper.cpp$(DependSuffix): g2o/g2o/stuff/sparse_helper.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/stuff_sparse_helper.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/stuff_sparse_helper.cpp$(DependSuffix) -MM "g2o/g2o/stuff/sparse_helper.cpp"

$(IntermediateDirectory)/stuff_sparse_helper.cpp$(PreprocessSuffix): g2o/g2o/stuff/sparse_helper.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/stuff_sparse_helper.cpp$(PreprocessSuffix) "g2o/g2o/stuff/sparse_helper.cpp"

$(IntermediateDirectory)/core_matrix_structure.cpp$(ObjectSuffix): g2o/g2o/core/matrix_structure.cpp $(IntermediateDirectory)/core_matrix_structure.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/matrix_structure.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_matrix_structure.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_matrix_structure.cpp$(DependSuffix): g2o/g2o/core/matrix_structure.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_matrix_structure.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_matrix_structure.cpp$(DependSuffix) -MM "g2o/g2o/core/matrix_structure.cpp"

$(IntermediateDirectory)/core_matrix_structure.cpp$(PreprocessSuffix): g2o/g2o/core/matrix_structure.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_matrix_structure.cpp$(PreprocessSuffix) "g2o/g2o/core/matrix_structure.cpp"

$(IntermediateDirectory)/core_sparse_optimizer.cpp$(ObjectSuffix): g2o/g2o/core/sparse_optimizer.cpp $(IntermediateDirectory)/core_sparse_optimizer.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/sparse_optimizer.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_sparse_optimizer.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_sparse_optimizer.cpp$(DependSuffix): g2o/g2o/core/sparse_optimizer.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_sparse_optimizer.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_sparse_optimizer.cpp$(DependSuffix) -MM "g2o/g2o/core/sparse_optimizer.cpp"

$(IntermediateDirectory)/core_sparse_optimizer.cpp$(PreprocessSuffix): g2o/g2o/core/sparse_optimizer.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_sparse_optimizer.cpp$(PreprocessSuffix) "g2o/g2o/core/sparse_optimizer.cpp"

$(IntermediateDirectory)/core_hyper_graph_action.cpp$(ObjectSuffix): g2o/g2o/core/hyper_graph_action.cpp $(IntermediateDirectory)/core_hyper_graph_action.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/hyper_graph_action.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_hyper_graph_action.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_hyper_graph_action.cpp$(DependSuffix): g2o/g2o/core/hyper_graph_action.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_hyper_graph_action.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_hyper_graph_action.cpp$(DependSuffix) -MM "g2o/g2o/core/hyper_graph_action.cpp"

$(IntermediateDirectory)/core_hyper_graph_action.cpp$(PreprocessSuffix): g2o/g2o/core/hyper_graph_action.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_hyper_graph_action.cpp$(PreprocessSuffix) "g2o/g2o/core/hyper_graph_action.cpp"

$(IntermediateDirectory)/core_hyper_graph.cpp$(ObjectSuffix): g2o/g2o/core/hyper_graph.cpp $(IntermediateDirectory)/core_hyper_graph.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/hyper_graph.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_hyper_graph.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_hyper_graph.cpp$(DependSuffix): g2o/g2o/core/hyper_graph.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_hyper_graph.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_hyper_graph.cpp$(DependSuffix) -MM "g2o/g2o/core/hyper_graph.cpp"

$(IntermediateDirectory)/core_hyper_graph.cpp$(PreprocessSuffix): g2o/g2o/core/hyper_graph.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_hyper_graph.cpp$(PreprocessSuffix) "g2o/g2o/core/hyper_graph.cpp"

$(IntermediateDirectory)/core_cache.cpp$(ObjectSuffix): g2o/g2o/core/cache.cpp $(IntermediateDirectory)/core_cache.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/cache.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_cache.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_cache.cpp$(DependSuffix): g2o/g2o/core/cache.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_cache.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_cache.cpp$(DependSuffix) -MM "g2o/g2o/core/cache.cpp"

$(IntermediateDirectory)/core_cache.cpp$(PreprocessSuffix): g2o/g2o/core/cache.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_cache.cpp$(PreprocessSuffix) "g2o/g2o/core/cache.cpp"

$(IntermediateDirectory)/core_optimization_algorithm_factory.cpp$(ObjectSuffix): g2o/g2o/core/optimization_algorithm_factory.cpp $(IntermediateDirectory)/core_optimization_algorithm_factory.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/optimization_algorithm_factory.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_optimization_algorithm_factory.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_optimization_algorithm_factory.cpp$(DependSuffix): g2o/g2o/core/optimization_algorithm_factory.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_optimization_algorithm_factory.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_optimization_algorithm_factory.cpp$(DependSuffix) -MM "g2o/g2o/core/optimization_algorithm_factory.cpp"

$(IntermediateDirectory)/core_optimization_algorithm_factory.cpp$(PreprocessSuffix): g2o/g2o/core/optimization_algorithm_factory.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_optimization_algorithm_factory.cpp$(PreprocessSuffix) "g2o/g2o/core/optimization_algorithm_factory.cpp"

$(IntermediateDirectory)/core_optimization_algorithm.cpp$(ObjectSuffix): g2o/g2o/core/optimization_algorithm.cpp $(IntermediateDirectory)/core_optimization_algorithm.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/optimization_algorithm.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_optimization_algorithm.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_optimization_algorithm.cpp$(DependSuffix): g2o/g2o/core/optimization_algorithm.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_optimization_algorithm.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_optimization_algorithm.cpp$(DependSuffix) -MM "g2o/g2o/core/optimization_algorithm.cpp"

$(IntermediateDirectory)/core_optimization_algorithm.cpp$(PreprocessSuffix): g2o/g2o/core/optimization_algorithm.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_optimization_algorithm.cpp$(PreprocessSuffix) "g2o/g2o/core/optimization_algorithm.cpp"

$(IntermediateDirectory)/core_optimization_algorithm_with_hessian.cpp$(ObjectSuffix): g2o/g2o/core/optimization_algorithm_with_hessian.cpp $(IntermediateDirectory)/core_optimization_algorithm_with_hessian.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/optimization_algorithm_with_hessian.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_optimization_algorithm_with_hessian.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_optimization_algorithm_with_hessian.cpp$(DependSuffix): g2o/g2o/core/optimization_algorithm_with_hessian.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_optimization_algorithm_with_hessian.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_optimization_algorithm_with_hessian.cpp$(DependSuffix) -MM "g2o/g2o/core/optimization_algorithm_with_hessian.cpp"

$(IntermediateDirectory)/core_optimization_algorithm_with_hessian.cpp$(PreprocessSuffix): g2o/g2o/core/optimization_algorithm_with_hessian.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_optimization_algorithm_with_hessian.cpp$(PreprocessSuffix) "g2o/g2o/core/optimization_algorithm_with_hessian.cpp"

$(IntermediateDirectory)/core_robust_kernel.cpp$(ObjectSuffix): g2o/g2o/core/robust_kernel.cpp $(IntermediateDirectory)/core_robust_kernel.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/robust_kernel.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_robust_kernel.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_robust_kernel.cpp$(DependSuffix): g2o/g2o/core/robust_kernel.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_robust_kernel.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_robust_kernel.cpp$(DependSuffix) -MM "g2o/g2o/core/robust_kernel.cpp"

$(IntermediateDirectory)/core_robust_kernel.cpp$(PreprocessSuffix): g2o/g2o/core/robust_kernel.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_robust_kernel.cpp$(PreprocessSuffix) "g2o/g2o/core/robust_kernel.cpp"

$(IntermediateDirectory)/core_estimate_propagator.cpp$(ObjectSuffix): g2o/g2o/core/estimate_propagator.cpp $(IntermediateDirectory)/core_estimate_propagator.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/estimate_propagator.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_estimate_propagator.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_estimate_propagator.cpp$(DependSuffix): g2o/g2o/core/estimate_propagator.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_estimate_propagator.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_estimate_propagator.cpp$(DependSuffix) -MM "g2o/g2o/core/estimate_propagator.cpp"

$(IntermediateDirectory)/core_estimate_propagator.cpp$(PreprocessSuffix): g2o/g2o/core/estimate_propagator.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_estimate_propagator.cpp$(PreprocessSuffix) "g2o/g2o/core/estimate_propagator.cpp"

$(IntermediateDirectory)/core_sparse_optimizer_terminate_action.cpp$(ObjectSuffix): g2o/g2o/core/sparse_optimizer_terminate_action.cpp $(IntermediateDirectory)/core_sparse_optimizer_terminate_action.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/sparse_optimizer_terminate_action.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_sparse_optimizer_terminate_action.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_sparse_optimizer_terminate_action.cpp$(DependSuffix): g2o/g2o/core/sparse_optimizer_terminate_action.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_sparse_optimizer_terminate_action.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_sparse_optimizer_terminate_action.cpp$(DependSuffix) -MM "g2o/g2o/core/sparse_optimizer_terminate_action.cpp"

$(IntermediateDirectory)/core_sparse_optimizer_terminate_action.cpp$(PreprocessSuffix): g2o/g2o/core/sparse_optimizer_terminate_action.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_sparse_optimizer_terminate_action.cpp$(PreprocessSuffix) "g2o/g2o/core/sparse_optimizer_terminate_action.cpp"

$(IntermediateDirectory)/core_solver.cpp$(ObjectSuffix): g2o/g2o/core/solver.cpp $(IntermediateDirectory)/core_solver.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/solver.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_solver.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_solver.cpp$(DependSuffix): g2o/g2o/core/solver.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_solver.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_solver.cpp$(DependSuffix) -MM "g2o/g2o/core/solver.cpp"

$(IntermediateDirectory)/core_solver.cpp$(PreprocessSuffix): g2o/g2o/core/solver.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_solver.cpp$(PreprocessSuffix) "g2o/g2o/core/solver.cpp"

$(IntermediateDirectory)/core_factory.cpp$(ObjectSuffix): g2o/g2o/core/factory.cpp $(IntermediateDirectory)/core_factory.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/factory.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_factory.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_factory.cpp$(DependSuffix): g2o/g2o/core/factory.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_factory.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_factory.cpp$(DependSuffix) -MM "g2o/g2o/core/factory.cpp"

$(IntermediateDirectory)/core_factory.cpp$(PreprocessSuffix): g2o/g2o/core/factory.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_factory.cpp$(PreprocessSuffix) "g2o/g2o/core/factory.cpp"

$(IntermediateDirectory)/core_sparse_block_matrix_test.cpp$(ObjectSuffix): g2o/g2o/core/sparse_block_matrix_test.cpp $(IntermediateDirectory)/core_sparse_block_matrix_test.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/sparse_block_matrix_test.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_sparse_block_matrix_test.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_sparse_block_matrix_test.cpp$(DependSuffix): g2o/g2o/core/sparse_block_matrix_test.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_sparse_block_matrix_test.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_sparse_block_matrix_test.cpp$(DependSuffix) -MM "g2o/g2o/core/sparse_block_matrix_test.cpp"

$(IntermediateDirectory)/core_sparse_block_matrix_test.cpp$(PreprocessSuffix): g2o/g2o/core/sparse_block_matrix_test.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_sparse_block_matrix_test.cpp$(PreprocessSuffix) "g2o/g2o/core/sparse_block_matrix_test.cpp"

$(IntermediateDirectory)/core_optimization_algorithm_levenberg.cpp$(ObjectSuffix): g2o/g2o/core/optimization_algorithm_levenberg.cpp $(IntermediateDirectory)/core_optimization_algorithm_levenberg.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/optimization_algorithm_levenberg.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_optimization_algorithm_levenberg.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_optimization_algorithm_levenberg.cpp$(DependSuffix): g2o/g2o/core/optimization_algorithm_levenberg.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_optimization_algorithm_levenberg.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_optimization_algorithm_levenberg.cpp$(DependSuffix) -MM "g2o/g2o/core/optimization_algorithm_levenberg.cpp"

$(IntermediateDirectory)/core_optimization_algorithm_levenberg.cpp$(PreprocessSuffix): g2o/g2o/core/optimization_algorithm_levenberg.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_optimization_algorithm_levenberg.cpp$(PreprocessSuffix) "g2o/g2o/core/optimization_algorithm_levenberg.cpp"

$(IntermediateDirectory)/core_marginal_covariance_cholesky.cpp$(ObjectSuffix): g2o/g2o/core/marginal_covariance_cholesky.cpp $(IntermediateDirectory)/core_marginal_covariance_cholesky.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/marginal_covariance_cholesky.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_marginal_covariance_cholesky.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_marginal_covariance_cholesky.cpp$(DependSuffix): g2o/g2o/core/marginal_covariance_cholesky.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_marginal_covariance_cholesky.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_marginal_covariance_cholesky.cpp$(DependSuffix) -MM "g2o/g2o/core/marginal_covariance_cholesky.cpp"

$(IntermediateDirectory)/core_marginal_covariance_cholesky.cpp$(PreprocessSuffix): g2o/g2o/core/marginal_covariance_cholesky.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_marginal_covariance_cholesky.cpp$(PreprocessSuffix) "g2o/g2o/core/marginal_covariance_cholesky.cpp"

$(IntermediateDirectory)/core_robust_kernel_factory.cpp$(ObjectSuffix): g2o/g2o/core/robust_kernel_factory.cpp $(IntermediateDirectory)/core_robust_kernel_factory.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/robust_kernel_factory.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_robust_kernel_factory.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_robust_kernel_factory.cpp$(DependSuffix): g2o/g2o/core/robust_kernel_factory.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_robust_kernel_factory.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_robust_kernel_factory.cpp$(DependSuffix) -MM "g2o/g2o/core/robust_kernel_factory.cpp"

$(IntermediateDirectory)/core_robust_kernel_factory.cpp$(PreprocessSuffix): g2o/g2o/core/robust_kernel_factory.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_robust_kernel_factory.cpp$(PreprocessSuffix) "g2o/g2o/core/robust_kernel_factory.cpp"

$(IntermediateDirectory)/core_parameter.cpp$(ObjectSuffix): g2o/g2o/core/parameter.cpp $(IntermediateDirectory)/core_parameter.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/parameter.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_parameter.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_parameter.cpp$(DependSuffix): g2o/g2o/core/parameter.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_parameter.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_parameter.cpp$(DependSuffix) -MM "g2o/g2o/core/parameter.cpp"

$(IntermediateDirectory)/core_parameter.cpp$(PreprocessSuffix): g2o/g2o/core/parameter.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_parameter.cpp$(PreprocessSuffix) "g2o/g2o/core/parameter.cpp"

$(IntermediateDirectory)/core_optimizable_graph.cpp$(ObjectSuffix): g2o/g2o/core/optimizable_graph.cpp $(IntermediateDirectory)/core_optimizable_graph.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/optimizable_graph.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_optimizable_graph.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_optimizable_graph.cpp$(DependSuffix): g2o/g2o/core/optimizable_graph.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_optimizable_graph.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_optimizable_graph.cpp$(DependSuffix) -MM "g2o/g2o/core/optimizable_graph.cpp"

$(IntermediateDirectory)/core_optimizable_graph.cpp$(PreprocessSuffix): g2o/g2o/core/optimizable_graph.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_optimizable_graph.cpp$(PreprocessSuffix) "g2o/g2o/core/optimizable_graph.cpp"

$(IntermediateDirectory)/core_jacobian_workspace.cpp$(ObjectSuffix): g2o/g2o/core/jacobian_workspace.cpp $(IntermediateDirectory)/core_jacobian_workspace.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/jacobian_workspace.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_jacobian_workspace.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_jacobian_workspace.cpp$(DependSuffix): g2o/g2o/core/jacobian_workspace.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_jacobian_workspace.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_jacobian_workspace.cpp$(DependSuffix) -MM "g2o/g2o/core/jacobian_workspace.cpp"

$(IntermediateDirectory)/core_jacobian_workspace.cpp$(PreprocessSuffix): g2o/g2o/core/jacobian_workspace.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_jacobian_workspace.cpp$(PreprocessSuffix) "g2o/g2o/core/jacobian_workspace.cpp"

$(IntermediateDirectory)/core_optimization_algorithm_dogleg.cpp$(ObjectSuffix): g2o/g2o/core/optimization_algorithm_dogleg.cpp $(IntermediateDirectory)/core_optimization_algorithm_dogleg.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/optimization_algorithm_dogleg.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_optimization_algorithm_dogleg.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_optimization_algorithm_dogleg.cpp$(DependSuffix): g2o/g2o/core/optimization_algorithm_dogleg.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_optimization_algorithm_dogleg.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_optimization_algorithm_dogleg.cpp$(DependSuffix) -MM "g2o/g2o/core/optimization_algorithm_dogleg.cpp"

$(IntermediateDirectory)/core_optimization_algorithm_dogleg.cpp$(PreprocessSuffix): g2o/g2o/core/optimization_algorithm_dogleg.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_optimization_algorithm_dogleg.cpp$(PreprocessSuffix) "g2o/g2o/core/optimization_algorithm_dogleg.cpp"

$(IntermediateDirectory)/core_hyper_dijkstra.cpp$(ObjectSuffix): g2o/g2o/core/hyper_dijkstra.cpp $(IntermediateDirectory)/core_hyper_dijkstra.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/hyper_dijkstra.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_hyper_dijkstra.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_hyper_dijkstra.cpp$(DependSuffix): g2o/g2o/core/hyper_dijkstra.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_hyper_dijkstra.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_hyper_dijkstra.cpp$(DependSuffix) -MM "g2o/g2o/core/hyper_dijkstra.cpp"

$(IntermediateDirectory)/core_hyper_dijkstra.cpp$(PreprocessSuffix): g2o/g2o/core/hyper_dijkstra.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_hyper_dijkstra.cpp$(PreprocessSuffix) "g2o/g2o/core/hyper_dijkstra.cpp"

$(IntermediateDirectory)/core_parameter_container.cpp$(ObjectSuffix): g2o/g2o/core/parameter_container.cpp $(IntermediateDirectory)/core_parameter_container.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/parameter_container.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_parameter_container.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_parameter_container.cpp$(DependSuffix): g2o/g2o/core/parameter_container.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_parameter_container.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_parameter_container.cpp$(DependSuffix) -MM "g2o/g2o/core/parameter_container.cpp"

$(IntermediateDirectory)/core_parameter_container.cpp$(PreprocessSuffix): g2o/g2o/core/parameter_container.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_parameter_container.cpp$(PreprocessSuffix) "g2o/g2o/core/parameter_container.cpp"

$(IntermediateDirectory)/core_robust_kernel_impl.cpp$(ObjectSuffix): g2o/g2o/core/robust_kernel_impl.cpp $(IntermediateDirectory)/core_robust_kernel_impl.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/robust_kernel_impl.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_robust_kernel_impl.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_robust_kernel_impl.cpp$(DependSuffix): g2o/g2o/core/robust_kernel_impl.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_robust_kernel_impl.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_robust_kernel_impl.cpp$(DependSuffix) -MM "g2o/g2o/core/robust_kernel_impl.cpp"

$(IntermediateDirectory)/core_robust_kernel_impl.cpp$(PreprocessSuffix): g2o/g2o/core/robust_kernel_impl.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_robust_kernel_impl.cpp$(PreprocessSuffix) "g2o/g2o/core/robust_kernel_impl.cpp"

$(IntermediateDirectory)/core_optimization_algorithm_gauss_newton.cpp$(ObjectSuffix): g2o/g2o/core/optimization_algorithm_gauss_newton.cpp $(IntermediateDirectory)/core_optimization_algorithm_gauss_newton.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/optimization_algorithm_gauss_newton.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_optimization_algorithm_gauss_newton.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_optimization_algorithm_gauss_newton.cpp$(DependSuffix): g2o/g2o/core/optimization_algorithm_gauss_newton.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_optimization_algorithm_gauss_newton.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_optimization_algorithm_gauss_newton.cpp$(DependSuffix) -MM "g2o/g2o/core/optimization_algorithm_gauss_newton.cpp"

$(IntermediateDirectory)/core_optimization_algorithm_gauss_newton.cpp$(PreprocessSuffix): g2o/g2o/core/optimization_algorithm_gauss_newton.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_optimization_algorithm_gauss_newton.cpp$(PreprocessSuffix) "g2o/g2o/core/optimization_algorithm_gauss_newton.cpp"

$(IntermediateDirectory)/core_batch_stats.cpp$(ObjectSuffix): g2o/g2o/core/batch_stats.cpp $(IntermediateDirectory)/core_batch_stats.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/core/batch_stats.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/core_batch_stats.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/core_batch_stats.cpp$(DependSuffix): g2o/g2o/core/batch_stats.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/core_batch_stats.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/core_batch_stats.cpp$(DependSuffix) -MM "g2o/g2o/core/batch_stats.cpp"

$(IntermediateDirectory)/core_batch_stats.cpp$(PreprocessSuffix): g2o/g2o/core/batch_stats.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/core_batch_stats.cpp$(PreprocessSuffix) "g2o/g2o/core/batch_stats.cpp"

$(IntermediateDirectory)/dutils_Random.cpp$(ObjectSuffix): dbow2/src/dutils/Random.cpp $(IntermediateDirectory)/dutils_Random.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/dbow2/src/dutils/Random.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dutils_Random.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dutils_Random.cpp$(DependSuffix): dbow2/src/dutils/Random.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dutils_Random.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dutils_Random.cpp$(DependSuffix) -MM "dbow2/src/dutils/Random.cpp"

$(IntermediateDirectory)/dutils_Random.cpp$(PreprocessSuffix): dbow2/src/dutils/Random.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dutils_Random.cpp$(PreprocessSuffix) "dbow2/src/dutils/Random.cpp"

$(IntermediateDirectory)/dutils_Timestamp.cpp$(ObjectSuffix): dbow2/src/dutils/Timestamp.cpp $(IntermediateDirectory)/dutils_Timestamp.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/dbow2/src/dutils/Timestamp.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dutils_Timestamp.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dutils_Timestamp.cpp$(DependSuffix): dbow2/src/dutils/Timestamp.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dutils_Timestamp.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dutils_Timestamp.cpp$(DependSuffix) -MM "dbow2/src/dutils/Timestamp.cpp"

$(IntermediateDirectory)/dutils_Timestamp.cpp$(PreprocessSuffix): dbow2/src/dutils/Timestamp.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dutils_Timestamp.cpp$(PreprocessSuffix) "dbow2/src/dutils/Timestamp.cpp"

$(IntermediateDirectory)/cholmod_solver_cholmod.cpp$(ObjectSuffix): g2o/g2o/solvers/cholmod/solver_cholmod.cpp $(IntermediateDirectory)/cholmod_solver_cholmod.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/solvers/cholmod/solver_cholmod.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/cholmod_solver_cholmod.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/cholmod_solver_cholmod.cpp$(DependSuffix): g2o/g2o/solvers/cholmod/solver_cholmod.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/cholmod_solver_cholmod.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/cholmod_solver_cholmod.cpp$(DependSuffix) -MM "g2o/g2o/solvers/cholmod/solver_cholmod.cpp"

$(IntermediateDirectory)/cholmod_solver_cholmod.cpp$(PreprocessSuffix): g2o/g2o/solvers/cholmod/solver_cholmod.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/cholmod_solver_cholmod.cpp$(PreprocessSuffix) "g2o/g2o/solvers/cholmod/solver_cholmod.cpp"

$(IntermediateDirectory)/dense_solver_dense.cpp$(ObjectSuffix): g2o/g2o/solvers/dense/solver_dense.cpp $(IntermediateDirectory)/dense_solver_dense.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/solvers/dense/solver_dense.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/dense_solver_dense.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/dense_solver_dense.cpp$(DependSuffix): g2o/g2o/solvers/dense/solver_dense.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/dense_solver_dense.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/dense_solver_dense.cpp$(DependSuffix) -MM "g2o/g2o/solvers/dense/solver_dense.cpp"

$(IntermediateDirectory)/dense_solver_dense.cpp$(PreprocessSuffix): g2o/g2o/solvers/dense/solver_dense.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/dense_solver_dense.cpp$(PreprocessSuffix) "g2o/g2o/solvers/dense/solver_dense.cpp"

$(IntermediateDirectory)/sba_types_six_dof_expmap.cpp$(ObjectSuffix): g2o/g2o/types/sba/types_six_dof_expmap.cpp $(IntermediateDirectory)/sba_types_six_dof_expmap.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/sba/types_six_dof_expmap.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sba_types_six_dof_expmap.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sba_types_six_dof_expmap.cpp$(DependSuffix): g2o/g2o/types/sba/types_six_dof_expmap.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sba_types_six_dof_expmap.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/sba_types_six_dof_expmap.cpp$(DependSuffix) -MM "g2o/g2o/types/sba/types_six_dof_expmap.cpp"

$(IntermediateDirectory)/sba_types_six_dof_expmap.cpp$(PreprocessSuffix): g2o/g2o/types/sba/types_six_dof_expmap.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sba_types_six_dof_expmap.cpp$(PreprocessSuffix) "g2o/g2o/types/sba/types_six_dof_expmap.cpp"

$(IntermediateDirectory)/sba_types_sba.cpp$(ObjectSuffix): g2o/g2o/types/sba/types_sba.cpp $(IntermediateDirectory)/sba_types_sba.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/sba/types_sba.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sba_types_sba.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sba_types_sba.cpp$(DependSuffix): g2o/g2o/types/sba/types_sba.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sba_types_sba.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/sba_types_sba.cpp$(DependSuffix) -MM "g2o/g2o/types/sba/types_sba.cpp"

$(IntermediateDirectory)/sba_types_sba.cpp$(PreprocessSuffix): g2o/g2o/types/sba/types_sba.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sba_types_sba.cpp$(PreprocessSuffix) "g2o/g2o/types/sba/types_sba.cpp"

$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_disparity.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/edge_se3_pointxyz_disparity.cpp $(IntermediateDirectory)/slam3d_edge_se3_pointxyz_disparity.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/edge_se3_pointxyz_disparity.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_disparity.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_disparity.cpp$(DependSuffix): g2o/g2o/types/slam3d/edge_se3_pointxyz_disparity.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_disparity.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_disparity.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/edge_se3_pointxyz_disparity.cpp"

$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_disparity.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/edge_se3_pointxyz_disparity.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_edge_se3_pointxyz_disparity.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/edge_se3_pointxyz_disparity.cpp"

$(IntermediateDirectory)/slam3d_parameter_camera.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/parameter_camera.cpp $(IntermediateDirectory)/slam3d_parameter_camera.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/parameter_camera.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_parameter_camera.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_parameter_camera.cpp$(DependSuffix): g2o/g2o/types/slam3d/parameter_camera.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_parameter_camera.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_parameter_camera.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/parameter_camera.cpp"

$(IntermediateDirectory)/slam3d_parameter_camera.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/parameter_camera.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_parameter_camera.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/parameter_camera.cpp"

$(IntermediateDirectory)/slam3d_isometry3d_mappings.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/isometry3d_mappings.cpp $(IntermediateDirectory)/slam3d_isometry3d_mappings.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/isometry3d_mappings.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_isometry3d_mappings.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_isometry3d_mappings.cpp$(DependSuffix): g2o/g2o/types/slam3d/isometry3d_mappings.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_isometry3d_mappings.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_isometry3d_mappings.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/isometry3d_mappings.cpp"

$(IntermediateDirectory)/slam3d_isometry3d_mappings.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/isometry3d_mappings.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_isometry3d_mappings.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/isometry3d_mappings.cpp"

$(IntermediateDirectory)/slam3d_edge_se3.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/edge_se3.cpp $(IntermediateDirectory)/slam3d_edge_se3.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/edge_se3.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_edge_se3.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_edge_se3.cpp$(DependSuffix): g2o/g2o/types/slam3d/edge_se3.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_edge_se3.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_edge_se3.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/edge_se3.cpp"

$(IntermediateDirectory)/slam3d_edge_se3.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/edge_se3.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_edge_se3.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/edge_se3.cpp"

$(IntermediateDirectory)/slam3d_dquat2mat.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/dquat2mat.cpp $(IntermediateDirectory)/slam3d_dquat2mat.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/dquat2mat.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_dquat2mat.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_dquat2mat.cpp$(DependSuffix): g2o/g2o/types/slam3d/dquat2mat.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_dquat2mat.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_dquat2mat.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/dquat2mat.cpp"

$(IntermediateDirectory)/slam3d_dquat2mat.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/dquat2mat.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_dquat2mat.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/dquat2mat.cpp"

$(IntermediateDirectory)/slam3d_parameter_stereo_camera.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/parameter_stereo_camera.cpp $(IntermediateDirectory)/slam3d_parameter_stereo_camera.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/parameter_stereo_camera.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_parameter_stereo_camera.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_parameter_stereo_camera.cpp$(DependSuffix): g2o/g2o/types/slam3d/parameter_stereo_camera.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_parameter_stereo_camera.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_parameter_stereo_camera.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/parameter_stereo_camera.cpp"

$(IntermediateDirectory)/slam3d_parameter_stereo_camera.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/parameter_stereo_camera.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_parameter_stereo_camera.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/parameter_stereo_camera.cpp"

$(IntermediateDirectory)/slam3d_vertex_se3.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/vertex_se3.cpp $(IntermediateDirectory)/slam3d_vertex_se3.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/vertex_se3.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_vertex_se3.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_vertex_se3.cpp$(DependSuffix): g2o/g2o/types/slam3d/vertex_se3.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_vertex_se3.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_vertex_se3.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/vertex_se3.cpp"

$(IntermediateDirectory)/slam3d_vertex_se3.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/vertex_se3.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_vertex_se3.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/vertex_se3.cpp"

$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_depth.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/edge_se3_pointxyz_depth.cpp $(IntermediateDirectory)/slam3d_edge_se3_pointxyz_depth.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/edge_se3_pointxyz_depth.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_depth.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_depth.cpp$(DependSuffix): g2o/g2o/types/slam3d/edge_se3_pointxyz_depth.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_depth.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_depth.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/edge_se3_pointxyz_depth.cpp"

$(IntermediateDirectory)/slam3d_edge_se3_pointxyz_depth.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/edge_se3_pointxyz_depth.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_edge_se3_pointxyz_depth.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/edge_se3_pointxyz_depth.cpp"

$(IntermediateDirectory)/slam3d_parameter_se3_offset.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/parameter_se3_offset.cpp $(IntermediateDirectory)/slam3d_parameter_se3_offset.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/parameter_se3_offset.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_parameter_se3_offset.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_parameter_se3_offset.cpp$(DependSuffix): g2o/g2o/types/slam3d/parameter_se3_offset.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_parameter_se3_offset.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_parameter_se3_offset.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/parameter_se3_offset.cpp"

$(IntermediateDirectory)/slam3d_parameter_se3_offset.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/parameter_se3_offset.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_parameter_se3_offset.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/parameter_se3_offset.cpp"

$(IntermediateDirectory)/slam3d_edge_se3_offset.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/edge_se3_offset.cpp $(IntermediateDirectory)/slam3d_edge_se3_offset.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/edge_se3_offset.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_edge_se3_offset.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_edge_se3_offset.cpp$(DependSuffix): g2o/g2o/types/slam3d/edge_se3_offset.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_edge_se3_offset.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_edge_se3_offset.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/edge_se3_offset.cpp"

$(IntermediateDirectory)/slam3d_edge_se3_offset.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/edge_se3_offset.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_edge_se3_offset.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/edge_se3_offset.cpp"

$(IntermediateDirectory)/slam3d_edge_se3_prior.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/edge_se3_prior.cpp $(IntermediateDirectory)/slam3d_edge_se3_prior.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/edge_se3_prior.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_edge_se3_prior.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_edge_se3_prior.cpp$(DependSuffix): g2o/g2o/types/slam3d/edge_se3_prior.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_edge_se3_prior.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_edge_se3_prior.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/edge_se3_prior.cpp"

$(IntermediateDirectory)/slam3d_edge_se3_prior.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/edge_se3_prior.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_edge_se3_prior.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/edge_se3_prior.cpp"

$(IntermediateDirectory)/slam3d_vertex_pointxyz.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/vertex_pointxyz.cpp $(IntermediateDirectory)/slam3d_vertex_pointxyz.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/vertex_pointxyz.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_vertex_pointxyz.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_vertex_pointxyz.cpp$(DependSuffix): g2o/g2o/types/slam3d/vertex_pointxyz.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_vertex_pointxyz.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_vertex_pointxyz.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/vertex_pointxyz.cpp"

$(IntermediateDirectory)/slam3d_vertex_pointxyz.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/vertex_pointxyz.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_vertex_pointxyz.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/vertex_pointxyz.cpp"

$(IntermediateDirectory)/slam3d_isometry3d_gradients.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/isometry3d_gradients.cpp $(IntermediateDirectory)/slam3d_isometry3d_gradients.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/isometry3d_gradients.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_isometry3d_gradients.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_isometry3d_gradients.cpp$(DependSuffix): g2o/g2o/types/slam3d/isometry3d_gradients.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_isometry3d_gradients.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_isometry3d_gradients.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/isometry3d_gradients.cpp"

$(IntermediateDirectory)/slam3d_isometry3d_gradients.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/isometry3d_gradients.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_isometry3d_gradients.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/isometry3d_gradients.cpp"

$(IntermediateDirectory)/slam3d_edge_se3_pointxyz.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/edge_se3_pointxyz.cpp $(IntermediateDirectory)/slam3d_edge_se3_pointxyz.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/edge_se3_pointxyz.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_edge_se3_pointxyz.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_edge_se3_pointxyz.cpp$(DependSuffix): g2o/g2o/types/slam3d/edge_se3_pointxyz.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_edge_se3_pointxyz.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_edge_se3_pointxyz.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/edge_se3_pointxyz.cpp"

$(IntermediateDirectory)/slam3d_edge_se3_pointxyz.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/edge_se3_pointxyz.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_edge_se3_pointxyz.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/edge_se3_pointxyz.cpp"

$(IntermediateDirectory)/slam3d_dquat2mat_maxima_generated.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/dquat2mat_maxima_generated.cpp $(IntermediateDirectory)/slam3d_dquat2mat_maxima_generated.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/dquat2mat_maxima_generated.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_dquat2mat_maxima_generated.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_dquat2mat_maxima_generated.cpp$(DependSuffix): g2o/g2o/types/slam3d/dquat2mat_maxima_generated.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_dquat2mat_maxima_generated.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_dquat2mat_maxima_generated.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/dquat2mat_maxima_generated.cpp"

$(IntermediateDirectory)/slam3d_dquat2mat_maxima_generated.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/dquat2mat_maxima_generated.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_dquat2mat_maxima_generated.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/dquat2mat_maxima_generated.cpp"

$(IntermediateDirectory)/slam3d_types_slam3d.cpp$(ObjectSuffix): g2o/g2o/types/slam3d/types_slam3d.cpp $(IntermediateDirectory)/slam3d_types_slam3d.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/slam3d/types_slam3d.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/slam3d_types_slam3d.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/slam3d_types_slam3d.cpp$(DependSuffix): g2o/g2o/types/slam3d/types_slam3d.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/slam3d_types_slam3d.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/slam3d_types_slam3d.cpp$(DependSuffix) -MM "g2o/g2o/types/slam3d/types_slam3d.cpp"

$(IntermediateDirectory)/slam3d_types_slam3d.cpp$(PreprocessSuffix): g2o/g2o/types/slam3d/types_slam3d.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/slam3d_types_slam3d.cpp$(PreprocessSuffix) "g2o/g2o/types/slam3d/types_slam3d.cpp"

$(IntermediateDirectory)/sim3_types_seven_dof_expmap.cpp$(ObjectSuffix): g2o/g2o/types/sim3/types_seven_dof_expmap.cpp $(IntermediateDirectory)/sim3_types_seven_dof_expmap.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/patrick/catkin_ws_orb/src/orb_slam/g2o/g2o/types/sim3/types_seven_dof_expmap.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/sim3_types_seven_dof_expmap.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/sim3_types_seven_dof_expmap.cpp$(DependSuffix): g2o/g2o/types/sim3/types_seven_dof_expmap.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/sim3_types_seven_dof_expmap.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/sim3_types_seven_dof_expmap.cpp$(DependSuffix) -MM "g2o/g2o/types/sim3/types_seven_dof_expmap.cpp"

$(IntermediateDirectory)/sim3_types_seven_dof_expmap.cpp$(PreprocessSuffix): g2o/g2o/types/sim3/types_seven_dof_expmap.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/sim3_types_seven_dof_expmap.cpp$(PreprocessSuffix) "g2o/g2o/types/sim3/types_seven_dof_expmap.cpp"


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) -r ./Debug/


