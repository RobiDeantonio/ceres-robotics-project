import json
import os
import decorator
# import sys

# from qOSM.config import config

doTrace = False


# backend = config['backend']
backend = "PyQt5"
if backend == "PyQt5":
    from PyQt5.QtCore import pyqtSignal, QUrl
    from PyQt5.QtGui import QDesktopServices
    from PyQt5.QtNetwork import QNetworkDiskCache, QNetworkAccessManager
    from PyQt5.QtWebEngineWidgets import QWebEngineSettings
    from PyQt5.QtWebChannel import QWebChannel
    from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage
    from PyQt5.QtWidgets import QApplication

elif backend == "PyQt4":
    from PyQt4.QtCore import pyqtSignal, QUrl
    from PyQt4.QtGui import QDesktopServices, QApplication
    from PyQt4.QtNetwork import QNetworkDiskCache
    from PyQt4.QtWebKit import QWebPage, QWebView, QWebSettings

@decorator.decorator
def trace(function, *args, **k):
    """Decorates a function by tracing the begining and
    end of the function execution, if doTrace global is True"""

    if doTrace:
        print("> " + function.__name__, args, k)
    result = function(*args, **k)
    if doTrace:
        print("< " + function.__name__, args, k, "->", result)
    return result


class _LoggedPage(QWebEnginePage):
    @trace
    def javaScriptConsoleMessage(self, msg, line, source):
        print('JS: %s line %d: %s' % (source, line, msg))


class QOSM(QWebEngineView):
    mapMoved = pyqtSignal(float, float)
    mapClicked = pyqtSignal(float, float)
    mapRightClicked = pyqtSignal(float, float)
    mapDoubleClicked = pyqtSignal(float, float)

    markerMoved = pyqtSignal(str, float, float)
    markerClicked = pyqtSignal(str, float, float)
    markerDoubleClicked = pyqtSignal(str, float, float)
    markerRightClicked = pyqtSignal(str, float, float)

    def __init__(self, parent=None, debug=True):
        QWebEngineView.__init__(self, parent=parent)

        # self.page_ = QWebEnginePage()
        # self.setPage(self.page_)

        cache = QNetworkDiskCache()
        cache.setCacheDirectory("cache")
        # self.page().QNetworkAccessManager().setCache(cache)
        # self.page().QNetworkAccessManager()

        # if debug:
        #    QWebEngineSettings.globalSettings().setAttribute(
        #        QWebEngineSettings.DeveloperExtrasEnabled, True
        #    )

        self.initialized = False

        channel = QWebChannel(self)
        self.page().setWebChannel(channel)
        channel.registerObject("qtWidget", self)

        basePath = os.path.abspath(os.path.dirname(__file__))
        url = 'file://' + basePath + '/qOSM.html'
        self.load(QUrl(url))

        # self.page().setLinkDelegationPolicy(QWebEnginePage.DelegateAllLinks)

        self.loadFinished.connect(self.onLoadFinished)
        QDesktopServices.openUrl

    def onLoadFinished(self, ok):
        if ok:
            frame = self.page()
            frame.toHtml(self.callback)

        if not ok:
            print("Error initializing OpenStreetMap")

        self.initialized = True
        self.centerAt(4.638, -74.08523)
        self.setZoom(16)

    def callback(self, html):
        pass # print(str(html).encode('utf-8'))

    def waitUntilReady(self):
        while not self.initialized:
            QApplication.processEvents()

    def runScript(self, script):
        return self.page().runJavaScript(script)

    def centerAt(self, latitude, longitude):
        self.runScript("osm_setCenter({}, {})".format(latitude, longitude))

    def setZoom(self, zoom):
        self.runScript("osm_setZoom({})".format(zoom))

    def center(self):
        center = self.runScript("osm_getCenter()")
        return center['lat'], center['lng']

    def addMarker(self, key, latitude, longitude, **extra):
        return self.runScript("osm_addMarker(key={!r},"
                              "latitude= {}, "
                              "longitude= {}, {});".format(key, latitude, longitude, json.dumps(extra)))

    def moveMarker(self, key, latitude, longitude):
        self.runScript("osm_moveMarker(key={!r},"
                       "latitude= {}, "
                       "longitude= {});".format(key, latitude, longitude))

    def positionMarker(self, key):
        return tuple(self.runScript("osm_posMarker(key={!r});".format(key)))
