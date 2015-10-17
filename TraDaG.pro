TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

VPATH += ./src \
    ./include/TraDaG
INCLUDEPATH += ./include

SOURCES += main.cpp \
    ogrewindow.cpp

HEADERS += include/OgreBites/SdkCameraMan.h \
    ogrewindow.h

# OGRE integration
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
LIBS += -lOgreBulletCollisions -lOgreBulletDynamics
contains(QT_ARCH, x86_64) {
    LIBS += -L$$PWD/libs/OgreBullet/x86_64
}
else {
    LIBS += -L$$PWD/libs/OgreBullet/i386
}

# Boost integration (required by OGRE)
LIBS += -lboost_system

include(deployment.pri)
qtcAddDeployment()
