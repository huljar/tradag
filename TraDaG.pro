TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

VPATH += ./src \
    ./include/TraDaG
INCLUDEPATH += ./include

HEADERS += ogrewindow.h

SOURCES += main.cpp \
    ogrewindow.cpp

# OGRE integration
INCLUDEPATH += /usr/include/OGRE
LIBS += -lOgreMain -lOgreOverlay -L/usr/lib/i386-linux-gnu/OGRE-1.9.0

# OIS
INCLUDEPATH += /usr/include/ois
LIBS += -lOIS

# Bullet
INCLUDEPATH += /usr/include/bullet
LIBS += -lBulletDynamics -lBulletCollision -lLinearMath #-lBulletSoftBody

# OgreBullet
LIBS += -lOgreBulletCollisions -lOgreBulletDynamics -L$$PWD/libs/OgreBullet

# Boost integration
LIBS += -lboost_system

include(deployment.pri)
qtcAddDeployment()
