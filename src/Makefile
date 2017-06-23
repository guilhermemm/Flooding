#
# OMNeT++/OMNEST Makefile for Flooding
#
# This file was generated with the command:
#  opp_makemake -f --deep -I../../veins/src -I../../veins/src/veins/base/utils -I../../veins/src/veins/modules/application/ieee80211p -I../../veins/src/veins/modules/mac/ieee80211p -I../../veins/src/veins/modules/phy -I../../veins/src/veins/modules/messages -I../../veins/src/veins/modules/mobility/traci -I../../veins/src/veins/base/phyLayer -L../../veins/out/$$\(CONFIGNAME\)/src -lveins -KVEINS_PROJ=../../veins
#

# Name of target to be created (-o option)
TARGET = Flooding$(EXE_SUFFIX)

# User interface (uncomment one) (-u option)
USERIF_LIBS = $(ALL_ENV_LIBS) # that is, $(TKENV_LIBS) $(CMDENV_LIBS)
#USERIF_LIBS = $(CMDENV_LIBS)
#USERIF_LIBS = $(TKENV_LIBS)

# C++ include paths (with -I)
INCLUDE_PATH = \
    -I../../veins/src \
    -I../../veins/src/veins/base/utils \
    -I../../veins/src/veins/modules/application/ieee80211p \
    -I../../veins/src/veins/modules/mac/ieee80211p \
    -I../../veins/src/veins/modules/phy \
    -I../../veins/src/veins/modules/messages \
    -I../../veins/src/veins/modules/mobility/traci \
    -I../../veins/src/veins/base/phyLayer \
    -I. \
    -Imessages \
    -Irouting \
    -Irsu

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS = -L../../veins/out/$(CONFIGNAME)/src  -lveins
LIBS += -Wl,-rpath,`abspath ../../veins/out/$(CONFIGNAME)/src`

# Output directory
PROJECT_OUTPUT_DIR = ../out
PROJECTRELATIVE_PATH = src
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc and .msg files
OBJS = $O/routing/Flooding.o $O/rsu/RSUApplication.o $O/messages/DataMessage_m.o

# Message files
MSGFILES = \
    messages/DataMessage.msg

# Other makefile variables (-K)
VEINS_PROJ=../../veins

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppmain$D $(USERIF_LIBS) $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS)  $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating executable: $@
	$(Q)$(CXX) $(LDFLAGS) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS)

.PHONY: all clean cleanall depend msgheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f Flooding Flooding.exe libFlooding.so libFlooding.a libFlooding.dll libFlooding.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h
	$(Q)-rm -f messages/*_m.cc messages/*_m.h
	$(Q)-rm -f routing/*_m.cc routing/*_m.h
	$(Q)-rm -f rsu/*_m.cc rsu/*_m.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES)  ./*.cc messages/*.cc routing/*.cc rsu/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/messages/DataMessage_m.o: messages/DataMessage_m.cc \
	messages/DataMessage_m.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h
$O/routing/Flooding.o: routing/Flooding.cc \
	messages/DataMessage_m.h \
	routing/Flooding.h \
	$(VEINS_PROJ)/src/veins/base/connectionManager/BaseConnectionManager.h \
	$(VEINS_PROJ)/src/veins/base/connectionManager/ChannelAccess.h \
	$(VEINS_PROJ)/src/veins/base/connectionManager/NicEntry.h \
	$(VEINS_PROJ)/src/veins/base/messages/AirFrame_m.h \
	$(VEINS_PROJ)/src/veins/base/messages/ChannelSenseRequest_m.h \
	$(VEINS_PROJ)/src/veins/base/messages/MacPkt_m.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseApplLayer.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseBattery.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseLayer.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseMacLayer.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseMobility.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseModule.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseWorldUtility.h \
	$(VEINS_PROJ)/src/veins/base/modules/BatteryAccess.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/AnalogueModel.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/BaseDecider.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/BasePhyLayer.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/ChannelInfo.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/ChannelState.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/Decider.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/DeciderToPhyInterface.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/Interpolation.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/MacToPhyControlInfo.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/MacToPhyInterface.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/Mapping.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/MappingBase.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/MappingUtils.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/PhyToMacControlInfo.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/PhyUtils.h \
	$(VEINS_PROJ)/src/veins/base/phyLayer/Signal_.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/FindModule.h \
	$(VEINS_PROJ)/src/veins/base/utils/HostState.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/Move.h \
	$(VEINS_PROJ)/src/veins/base/utils/NetwToMacControlInfo.h \
	$(VEINS_PROJ)/src/veins/base/utils/PassedMessage.h \
	$(VEINS_PROJ)/src/veins/base/utils/SimpleAddress.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h \
	$(VEINS_PROJ)/src/veins/modules/analogueModel/JakesFading.h \
	$(VEINS_PROJ)/src/veins/modules/analogueModel/LogNormalShadowing.h \
	$(VEINS_PROJ)/src/veins/modules/analogueModel/SimplePathlossModel.h \
	$(VEINS_PROJ)/src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
	$(VEINS_PROJ)/src/veins/modules/mac/ieee80211p/Mac1609_4.h \
	$(VEINS_PROJ)/src/veins/modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h \
	$(VEINS_PROJ)/src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
	$(VEINS_PROJ)/src/veins/modules/messages/Mac80211Pkt_m.h \
	$(VEINS_PROJ)/src/veins/modules/messages/PhyControlMessage_m.h \
	$(VEINS_PROJ)/src/veins/modules/messages/WaveShortMessage_m.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIBuffer.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIColor.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCICommandInterface.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIConnection.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCICoord.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIMobility.h \
	$(VEINS_PROJ)/src/veins/modules/mobility/traci/TraCIScenarioManager.h \
	$(VEINS_PROJ)/src/veins/modules/obstacle/Obstacle.h \
	$(VEINS_PROJ)/src/veins/modules/obstacle/ObstacleControl.h \
	$(VEINS_PROJ)/src/veins/modules/phy/Decider80211p.h \
	$(VEINS_PROJ)/src/veins/modules/phy/Decider80211pToPhy80211pInterface.h \
	$(VEINS_PROJ)/src/veins/modules/phy/DeciderResult80211.h \
	$(VEINS_PROJ)/src/veins/modules/phy/PhyLayer80211p.h \
	$(VEINS_PROJ)/src/veins/modules/phy/SNRThresholdDecider.h \
	$(VEINS_PROJ)/src/veins/modules/utility/Consts80211p.h \
	$(VEINS_PROJ)/src/veins/modules/utility/ConstsPhy.h \
	$(VEINS_PROJ)/src/veins/modules/world/annotations/AnnotationManager.h
$O/rsu/RSUApplication.o: rsu/RSUApplication.cc \
	messages/DataMessage_m.h \
	rsu/RSUApplication.h \
	$(VEINS_PROJ)/src/veins/base/connectionManager/ChannelAccess.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseApplLayer.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseBattery.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseLayer.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseMobility.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseModule.h \
	$(VEINS_PROJ)/src/veins/base/modules/BaseWorldUtility.h \
	$(VEINS_PROJ)/src/veins/base/modules/BatteryAccess.h \
	$(VEINS_PROJ)/src/veins/base/utils/Coord.h \
	$(VEINS_PROJ)/src/veins/base/utils/FWMath.h \
	$(VEINS_PROJ)/src/veins/base/utils/FindModule.h \
	$(VEINS_PROJ)/src/veins/base/utils/HostState.h \
	$(VEINS_PROJ)/src/veins/base/utils/MiXiMDefs.h \
	$(VEINS_PROJ)/src/veins/base/utils/Move.h \
	$(VEINS_PROJ)/src/veins/base/utils/NetwToMacControlInfo.h \
	$(VEINS_PROJ)/src/veins/base/utils/PassedMessage.h \
	$(VEINS_PROJ)/src/veins/base/utils/SimpleAddress.h \
	$(VEINS_PROJ)/src/veins/base/utils/miximkerneldefs.h \
	$(VEINS_PROJ)/src/veins/modules/application/ieee80211p/BaseWaveApplLayer.h \
	$(VEINS_PROJ)/src/veins/modules/mac/ieee80211p/WaveAppToMac1609_4Interface.h \
	$(VEINS_PROJ)/src/veins/modules/messages/WaveShortMessage_m.h \
	$(VEINS_PROJ)/src/veins/modules/utility/Consts80211p.h

