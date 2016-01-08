TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

VPATH += ./src \
    ./include/TraDaG
INCLUDEPATH += ./include

SOURCES += main.cpp \
    ogrewindow.cpp \
    rgbdscene.cpp \
    tradagmain.cpp \
    util.cpp \
    imagelabeling.cpp \
    droppableobject.cpp \
    src/groundplane.cpp

HEADERS += include/OgreBites/SdkCameraMan.h \
    include/OgreBites/OgreRay.h \
    ogrewindow.h \
    rgbdscene.h \
    tradagmain.h \
    ransac.h \
    util.h \
    imagelabeling.h \
    droppableobject.h \
    include/TraDaG/groundplane.h

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
LIBS += -lboost_system -lboost_program_options

# OpenCV
LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui

include(deployment.pri)
qtcAddDeployment()
