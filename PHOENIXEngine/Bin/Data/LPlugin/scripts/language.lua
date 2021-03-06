-- language.lua

function l_AddLanguage()
	PX2_LM_APP:Clear()
	PX2_LM_APP:AddItem1("Launcher","加载器")
	PX2_LM_APP:AddItem1("Updater", "更新器")
	PX2_LM_APP:AddItem("Engine", "PHOENIXEngine", "PHOENIXEngine")
	PX2_LM_APP:AddItem("01Robot", "零一机器人", "ZERONERobot")
	PX2_LM_APP:AddItem1("CodePlay", "编程玩")
	PX2_LM_APP:AddItem1("SignIn","登录")
	PX2_LM_APP:AddItem1("Launch", "启动")
	PX2_LM_APP:AddItem1("Open", "打开")
	PX2_LM_APP:AddItem1("Package", "打包")
	PX2_LM_APP:AddItem1("Editor", "编辑器")
	PX2_LM_APP:AddItem1("GenProjInfo", "生成项目信息")
	PX2_LM_APP:AddItem("NewProj","创建", "New")
	PX2_LM_APP:AddItem("OpenProj","打开", "Open")
	PX2_LM_APP:AddItem1("GenPackage", "生成打包项目")
	PX2_LM_APP:AddItem1("Syn", "同步")
	PX2_LM_APP:AddItem1("SynAll", "同步所有")
	PX2_LM_APP:AddItem1("LoadProject", "加载项目")
	PX2_LM_APP:AddItem1("CloseProject", "关闭项目")
	PX2_LM_APP:AddItem("Refresh", "刷新", "REFRESH")
	PX2_LM_APP:AddItem1("Send", "发送")
	PX2_LM_APP:AddItem1("use the Facebook Signin","使用FaceBook登入")
	PX2_LM_APP:AddItem1("use the GooGle Signin","使用GooGle登入")
	PX2_LM_APP:AddItem("RememberMe", "记住我", "Remember me")
	PX2_LM_APP:AddItem1("Forget the passWord","忘记密码？")
	PX2_LM_APP:AddItem1("register","还没有账户，注册 或 稍后登入")
	PX2_LM_APP:AddItem1("Create new project","创建新项目")
	PX2_LM_APP:AddItem1("OK","确定")
	PX2_LM_APP:AddItem1("Cancel","取消")
	PX2_LM_APP:AddItem1("Login","登入")
	PX2_LM_APP:AddItem1("YouCreate", "天天块乐")
	PX2_LM_APP:AddItem("Testing","内测版", "TEST_V")
	PX2_LM_APP:AddItem1("The Project Created can not be empty","创建的项目不能为空")
	PX2_LM_APP:AddItem1("Updating", "更新中")
	PX2_LM_APP:AddItem("GetFromTaobao", "从商店获得", "get from shop TaoBao")
	PX2_LM_APP:AddItem("Intell-Toy", "智能玩具", "Intell-Toy")
	PX2_LM_APP:AddItem("Chinese", "中文", "中文")
	PX2_LM_APP:AddItem("English", "English", "English")
	PX2_LM_APP:AddItem1("Cloud", "Cloud")
	PX2_LM_APP:AddItem1("Regist", "注册")
	PX2_LM_APP:AddItem1("Login", "登录")
	PX2_LM_APP:AddItem1("ForgetPassword","忘记密码")
	PX2_LM_APP:AddItem1("Account", "账号")
	PX2_LM_APP:AddItem1("Password", "密码")
	PX2_LM_APP:AddItem1("LoginOut", "登出")
	PX2_LM_APP:AddItem("Projects", "项目  <项目可在manykit.com云端分享>", "PROJECT <Project can be shared in manykit.com>")
	PX2_LM_APP:AddItem1("Project", "项目")
	PX2_LM_APP:AddItem("NetSyn", "网络同步  <选中IP和项目，点击同步，将项目通过Wifi局域网，更新项目到设备>", "NETSYN <Select device ip & project, press 'Syn', push project to the device>")
	PX2_LM_APP:AddItem1("Plugins", "插件")
	PX2_LM_APP:AddItem("ManyKit", "许多米科技", "MANYKIT")
	PX2_LM_APP:AddItem1("Download", "下载")
	PX2_LM_APP:AddItem1("Upload", "上传")
	PX2_LM_APP:AddItem1("UploadTitle", "上传到云端")
	PX2_LM_APP:AddItem("UploadContent", "上传项目到云端，覆盖原有项目，确定上传?", "upload project to cloudserver, replace origin project, are you sure?")
	PX2_LM_APP:AddItem1("DownloadTitle", "从云端下载")
	PX2_LM_APP:AddItem("DownloadContent", "从云端下载，覆盖本地'Data'目录项目，确定下载?", "Download project from cloudserver to directory 'Data', are you sure?")
	PX2_LM_APP:AddItem1("Info", "注意")
	PX2_LM_APP:AddItem("NeedLogin", "请先登录账号！", "login please!")
end