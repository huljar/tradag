TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

# Detect debug mode
CONFIG(debug, debug|release) {
    DEFINES += _DEBUG
}
else {
    DEFINES += _BENCHMARK # enable benchmarks in release
}

VPATH += ./src \
    ./include/TraDaG \
    ./include/TraDaG/CVLDWrapper
INCLUDEPATH += ./include

SOURCES += main.cpp \
    OgreWindow.cpp \
    RGBDScene.cpp \
    Simulator.cpp \
    ImageLabeling.cpp \
    DroppableObject.cpp \
    GroundPlane.cpp \
    PlaneInfo.cpp \
    SceneAnalyzer.cpp \
    CVLDWrapper.cpp \
    CameraManager.cpp \
    KDTree.cpp \
    Benchmarker.cpp

HEADERS += include/OgreBites/SdkCameraMan.h \
    include/OgreBites/OgreRay.h \
    OgreWindow.h \
    RGBDScene.h \
    Simulator.h \
    Ransac.h \
    util.h \
    ImageLabeling.h \
    DroppableObject.h \
    GroundPlane.h \
    PlaneInfo.h \
    SceneAnalyzer.h \
    CVLDWrapper.h \
    CameraManager.h \
    debug.h \
    interop.h \
    KDTree.h \
    Benchmarker.h

# OGRE
INCLUDEPATH += /usr/include/OGRE
LIBS += -lOgreMain -lOgreOverlay
contains(QT_ARCH, x86_64) {
    LIBS += -L/usr/lib/x86_64-linux-gnu/OGRE-1.9.0
}
else {
    LIBS += -L/usr/lib/i386-linux-gnu/OGRE-1.9.0
}

# OIS
INCLUDEPATH += /usr/include/ois
LIBS += -lOIS

# Bullet
INCLUDEPATH += /usr/include/bullet
LIBS += -lBulletDynamics -lBulletCollision -lLinearMath# -lBulletSoftBody

# OgreBullet
INCLUDEPATH += include/OgreBullet/Dynamics include/OgreBullet/Collisions
LIBS += -lOgreBulletCollisions -lOgreBulletDynamics
contains(QT_ARCH, x86_64) {
    LIBS += -L$$PWD/libs/OgreBullet/x86_64
}
else {
    LIBS += -L$$PWD/libs/OgreBullet/i386
}

# Boost
LIBS += -lboost_system -lboost_filesystem -lboost_regex

# OpenCV
LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui

# Required only by main.cpp
LIBS += -lboost_program_options

include(deployment.pri)
qtcAddDeployment()
