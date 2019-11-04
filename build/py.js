var Module=typeof pyodide._module!=="undefined"?pyodide._module:{};Module.checkABI(1);if(!Module.expectedDataFileDownloads){Module.expectedDataFileDownloads=0;Module.finishedDataFileDownloads=0}Module.expectedDataFileDownloads++;(function(){var loadPackage=function(metadata){var PACKAGE_PATH;if(typeof window==="object"){PACKAGE_PATH=window["encodeURIComponent"](window.location.pathname.toString().substring(0,window.location.pathname.toString().lastIndexOf("/"))+"/")}else if(typeof location!=="undefined"){PACKAGE_PATH=encodeURIComponent(location.pathname.toString().substring(0,location.pathname.toString().lastIndexOf("/"))+"/")}else{throw"using preloaded data can only be done on a web page or in a web worker"}var PACKAGE_NAME="py.data";var REMOTE_PACKAGE_BASE="py.data";if(typeof Module["locateFilePackage"]==="function"&&!Module["locateFile"]){Module["locateFile"]=Module["locateFilePackage"];err("warning: you defined Module.locateFilePackage, that has been renamed to Module.locateFile (using your locateFilePackage for now)")}var REMOTE_PACKAGE_NAME=Module["locateFile"]?Module["locateFile"](REMOTE_PACKAGE_BASE,""):REMOTE_PACKAGE_BASE;var REMOTE_PACKAGE_SIZE=metadata.remote_package_size;var PACKAGE_UUID=metadata.package_uuid;function fetchRemotePackage(packageName,packageSize,callback,errback){var xhr=new XMLHttpRequest;xhr.open("GET",packageName,true);xhr.responseType="arraybuffer";xhr.onprogress=function(event){var url=packageName;var size=packageSize;if(event.total)size=event.total;if(event.loaded){if(!xhr.addedTotal){xhr.addedTotal=true;if(!Module.dataFileDownloads)Module.dataFileDownloads={};Module.dataFileDownloads[url]={loaded:event.loaded,total:size}}else{Module.dataFileDownloads[url].loaded=event.loaded}var total=0;var loaded=0;var num=0;for(var download in Module.dataFileDownloads){var data=Module.dataFileDownloads[download];total+=data.total;loaded+=data.loaded;num++}total=Math.ceil(total*Module.expectedDataFileDownloads/num);if(Module["setStatus"])Module["setStatus"]("Downloading data... ("+loaded+"/"+total+")")}else if(!Module.dataFileDownloads){if(Module["setStatus"])Module["setStatus"]("Downloading data...")}};xhr.onerror=function(event){throw new Error("NetworkError for: "+packageName)};xhr.onload=function(event){if(xhr.status==200||xhr.status==304||xhr.status==206||xhr.status==0&&xhr.response){var packageData=xhr.response;callback(packageData)}else{throw new Error(xhr.statusText+" : "+xhr.responseURL)}};xhr.send(null)}function handleError(error){console.error("package error:",error)}var fetchedCallback=null;var fetched=Module["getPreloadedPackage"]?Module["getPreloadedPackage"](REMOTE_PACKAGE_NAME,REMOTE_PACKAGE_SIZE):null;if(!fetched)fetchRemotePackage(REMOTE_PACKAGE_NAME,REMOTE_PACKAGE_SIZE,function(data){if(fetchedCallback){fetchedCallback(data);fetchedCallback=null}else{fetched=data}},handleError);function runWithFS(){function assert(check,msg){if(!check)throw msg+(new Error).stack}Module["FS_createPath"]("/","lib",true,true);Module["FS_createPath"]("/lib","python3.7",true,true);Module["FS_createPath"]("/lib/python3.7","site-packages",true,true);Module["FS_createPath"]("/lib/python3.7/site-packages","py",true,true);Module["FS_createPath"]("/lib/python3.7/site-packages/py","_code",true,true);Module["FS_createPath"]("/lib/python3.7/site-packages/py","_io",true,true);Module["FS_createPath"]("/lib/python3.7/site-packages/py","_log",true,true);Module["FS_createPath"]("/lib/python3.7/site-packages/py","_path",true,true);Module["FS_createPath"]("/lib/python3.7/site-packages/py","_process",true,true);Module["FS_createPath"]("/lib/python3.7/site-packages/py","_vendored_packages",true,true);Module["FS_createPath"]("/lib/python3.7/site-packages","py-1.5.4-py3.7.egg-info",true,true);function DataRequest(start,end,audio){this.start=start;this.end=end;this.audio=audio}DataRequest.prototype={requests:{},open:function(mode,name){this.name=name;this.requests[name]=this;Module["addRunDependency"]("fp "+this.name)},send:function(){},onload:function(){var byteArray=this.byteArray.subarray(this.start,this.end);this.finish(byteArray)},finish:function(byteArray){var that=this;Module["FS_createPreloadedFile"](this.name,null,byteArray,true,true,function(){Module["removeRunDependency"]("fp "+that.name)},function(){if(that.audio){Module["removeRunDependency"]("fp "+that.name)}else{err("Preloading file "+that.name+" failed")}},false,true);this.requests[this.name]=null}};function processPackageData(arrayBuffer){Module.finishedDataFileDownloads++;assert(arrayBuffer,"Loading data file failed.");assert(arrayBuffer instanceof ArrayBuffer,"bad input to processPackageData");var byteArray=new Uint8Array(arrayBuffer);var curr;var compressedData={data:null,cachedOffset:153225,cachedIndexes:[-1,-1],cachedChunks:[null,null],offsets:[0,1286,2144,3082,4080,5308,6378,7573,8863,10096,11501,12577,13797,15178,16290,17284,18095,19042,20191,21532,22636,23446,24232,25152,26065,27087,28299,29793,31088,32271,33503,34685,35929,37065,38332,39590,40720,41811,42901,44070,45271,46249,47199,48244,49296,50407,51544,52815,54123,55418,56666,57778,59031,60197,61131,62212,63533,64895,66058,67328,68575,69634,70607,71764,73028,74149,75161,76375,77640,78810,80008,81118,82160,83427,84532,85759,86847,88082,89281,90348,91517,92504,93684,94809,95857,96929,97964,99230,100263,101464,102636,103807,104909,105894,107263,108352,109437,110627,111653,112874,114262,115530,116798,117978,119039,120213,121532,122752,123991,125104,126324,127482,128707,129821,131075,132222,133548,134593,135627,136514,137571,138819,140029,141286,142436,143552,144839,145883,147001,148118,149061,150264,151532,152529],sizes:[1286,858,938,998,1228,1070,1195,1290,1233,1405,1076,1220,1381,1112,994,811,947,1149,1341,1104,810,786,920,913,1022,1212,1494,1295,1183,1232,1182,1244,1136,1267,1258,1130,1091,1090,1169,1201,978,950,1045,1052,1111,1137,1271,1308,1295,1248,1112,1253,1166,934,1081,1321,1362,1163,1270,1247,1059,973,1157,1264,1121,1012,1214,1265,1170,1198,1110,1042,1267,1105,1227,1088,1235,1199,1067,1169,987,1180,1125,1048,1072,1035,1266,1033,1201,1172,1171,1102,985,1369,1089,1085,1190,1026,1221,1388,1268,1268,1180,1061,1174,1319,1220,1239,1113,1220,1158,1225,1114,1254,1147,1326,1045,1034,887,1057,1248,1210,1257,1150,1116,1287,1044,1118,1117,943,1203,1268,997,696],successes:[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]};compressedData.data=byteArray;assert(typeof Module.LZ4==="object","LZ4 not present - was your app build with  -s LZ4=1  ?");Module.LZ4.loadPackage({metadata:metadata,compressedData:compressedData});Module["removeRunDependency"]("datafile_py.data")}Module["addRunDependency"]("datafile_py.data");if(!Module.preloadResults)Module.preloadResults={};Module.preloadResults[PACKAGE_NAME]={fromCache:false};if(fetched){processPackageData(fetched);fetched=null}else{fetchedCallback=processPackageData}}if(Module["calledRun"]){runWithFS()}else{if(!Module["preRun"])Module["preRun"]=[];Module["preRun"].push(runWithFS)}};loadPackage({files:[{start:0,audio:0,end:6022,filename:"/lib/python3.7/site-packages/py/__init__.py"},{start:6022,audio:0,end:6077,filename:"/lib/python3.7/site-packages/py/__metainfo.py"},{start:6077,audio:0,end:12598,filename:"/lib/python3.7/site-packages/py/_builtin.py"},{start:12598,audio:0,end:15515,filename:"/lib/python3.7/site-packages/py/_error.py"},{start:15515,audio:0,end:16146,filename:"/lib/python3.7/site-packages/py/_std.py"},{start:16146,audio:0,end:16262,filename:"/lib/python3.7/site-packages/py/_version.py"},{start:16262,audio:0,end:24626,filename:"/lib/python3.7/site-packages/py/_xmlgen.py"},{start:24626,audio:0,end:24848,filename:"/lib/python3.7/site-packages/py/test.py"},{start:24848,audio:0,end:24894,filename:"/lib/python3.7/site-packages/py/_code/__init__.py"},{start:24894,audio:0,end:36344,filename:"/lib/python3.7/site-packages/py/_code/_assertionnew.py"},{start:36344,audio:0,end:54213,filename:"/lib/python3.7/site-packages/py/_code/_assertionold.py"},{start:54213,audio:0,end:56978,filename:"/lib/python3.7/site-packages/py/_code/_py2traceback.py"},{start:56978,audio:0,end:60152,filename:"/lib/python3.7/site-packages/py/_code/assertion.py"},{start:60152,audio:0,end:87644,filename:"/lib/python3.7/site-packages/py/_code/code.py"},{start:87644,audio:0,end:101694,filename:"/lib/python3.7/site-packages/py/_code/source.py"},{start:101694,audio:0,end:101723,filename:"/lib/python3.7/site-packages/py/_io/__init__.py"},{start:101723,audio:0,end:113363,filename:"/lib/python3.7/site-packages/py/_io/capture.py"},{start:113363,audio:0,end:115846,filename:"/lib/python3.7/site-packages/py/_io/saferepr.py"},{start:115846,audio:0,end:129332,filename:"/lib/python3.7/site-packages/py/_io/terminalwriter.py"},{start:129332,audio:0,end:129406,filename:"/lib/python3.7/site-packages/py/_log/__init__.py"},{start:129406,audio:0,end:135409,filename:"/lib/python3.7/site-packages/py/_log/log.py"},{start:135409,audio:0,end:137974,filename:"/lib/python3.7/site-packages/py/_log/warning.py"},{start:137974,audio:0,end:138006,filename:"/lib/python3.7/site-packages/py/_path/__init__.py"},{start:138006,audio:0,end:141339,filename:"/lib/python3.7/site-packages/py/_path/cacheutil.py"},{start:141339,audio:0,end:155965,filename:"/lib/python3.7/site-packages/py/_path/common.py"},{start:155965,audio:0,end:191291,filename:"/lib/python3.7/site-packages/py/_path/local.py"},{start:191291,audio:0,end:206006,filename:"/lib/python3.7/site-packages/py/_path/svnurl.py"},{start:206006,audio:0,end:249831,filename:"/lib/python3.7/site-packages/py/_path/svnwc.py"},{start:249831,audio:0,end:249871,filename:"/lib/python3.7/site-packages/py/_process/__init__.py"},{start:249871,audio:0,end:251685,filename:"/lib/python3.7/site-packages/py/_process/cmdexec.py"},{start:251685,audio:0,end:255377,filename:"/lib/python3.7/site-packages/py/_process/forkedfunc.py"},{start:255377,audio:0,end:256025,filename:"/lib/python3.7/site-packages/py/_process/killproc.py"},{start:256025,audio:0,end:256025,filename:"/lib/python3.7/site-packages/py/_vendored_packages/__init__.py"},{start:256025,audio:0,end:262445,filename:"/lib/python3.7/site-packages/py/_vendored_packages/apipkg.py"},{start:262445,audio:0,end:267653,filename:"/lib/python3.7/site-packages/py/_vendored_packages/iniconfig.py"},{start:267653,audio:0,end:270725,filename:"/lib/python3.7/site-packages/py-1.5.4-py3.7.egg-info/PKG-INFO"},{start:270725,audio:0,end:274280,filename:"/lib/python3.7/site-packages/py-1.5.4-py3.7.egg-info/SOURCES.txt"},{start:274280,audio:0,end:274281,filename:"/lib/python3.7/site-packages/py-1.5.4-py3.7.egg-info/dependency_links.txt"},{start:274281,audio:0,end:274282,filename:"/lib/python3.7/site-packages/py-1.5.4-py3.7.egg-info/not-zip-safe"},{start:274282,audio:0,end:274285,filename:"/lib/python3.7/site-packages/py-1.5.4-py3.7.egg-info/top_level.txt"}],remote_package_size:157321,package_uuid:"be16f065-7992-45da-bac6-90e05b9b489a"})})();