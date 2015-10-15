TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

VPATH += ./src \
    ./include/TraDaG
INCLUDEPATH += ./include

SOURCES += main.cpp

HEADERS +=

# OGRE integration
INCLUDEPATH += /usr/include/OGRE
LIBS += -lOgreMain -L/usr/lib/i386-linux-gnu/OGRE-1.9.0

# Bullet
INCLUDEPATH += /usr/include/bullet
LIBS += -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath

# OgreBullet
LIBS += -lOgreBulletCollisions -lOgreBulletDynamics -Llibs/OgreBullet

# Boost integration
LIBS += -lboost_system

include(deployment.pri)
qtcAddDeployment()
