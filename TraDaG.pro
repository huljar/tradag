TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

# Detect debug mode
CONFIG(debug, debug|release) {
    DEFINES += _DEBUG
}

VPATH += ./src \
    ./include/TraDaG \
    ./include/CVLDWrapper
INCLUDEPATH += ./include

SOURCES += main.cpp \
    OgreWindow.cpp \
    RGBDScene.cpp \
    TradagMain.cpp \
    util.cpp \
    ImageLabeling.cpp \
    DroppableObject.cpp \
    GroundPlane.cpp \
    ImageAnalyzer.cpp \
    CVLDWrapper.cpp

HEADERS += include/OgreBites/SdkCameraMan.h \
    include/OgreBites/OgreRay.h \
    OgreWindow.h \
    RGBDScene.h \
    TradagMain.h \
    Ransac.h \
    util.h \
    ImageLabeling.h \
    DroppableObject.h \
    GroundPlane.h \
    ImageAnalyzer.h \
    CVLDWrapper.h \
    debug.h

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
LIBS += -lboost_system -lboost_program_options -lboost_filesystem

# OpenCV
LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui

include(deployment.pri)
qtcAddDeployment()
