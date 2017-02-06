#include "App.h"

#include <nanogui/label.h>
#include <nanogui/layout.h>

#include "scenarios/BirdScenario.h"
#include "scenarios/BipedScenario.h"

const double gFPS = 30;

const cApp::eScene gDefaultScene = cApp::eSceneCurve;

cApp::cApp(int w, int h, const std::string& title) : nanogui::Screen(Eigen::Vector2i(w, h), title)
{
	mGUIWindow = nullptr;
	mSceneCombo = nullptr;
	mPlayButton = nullptr;
	mPlaybackSlider = nullptr;
	mPrevTime = 0;
	mEnableAnimation = true;
}

cApp::~cApp() 
{
	mScenario.reset();
}

void cApp::Init()
{
	cDrawUtil::InitDrawUtil();
	BuildScenario(gDefaultScene);
	BuildGUI();
}

bool cApp::keyboardEvent(int key, int scancode, int action, int modifiers) {
	if (Screen::keyboardEvent(key, scancode, action, modifiers))
		return true;
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		setVisible(false);
		return true;
	}
	return false;
}

bool cApp::mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers)
{
	if (nanogui::Screen::mouseButtonEvent(p, button, down, modifiers))
	{
		return true;
	}
	if (mScenario != nullptr)
	{
		return mScenario->MouseButtonEvent(p, button, down, modifiers);
	}
	return false;
}

bool cApp::mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers)
{
	if (nanogui::Screen::mouseMotionEvent(p, rel, button, modifiers))
	{
		return true;
	}
	if (mScenario != nullptr)
	{
		return mScenario->MouseMotionEvent(p, rel, button, modifiers);
	}
	return false;
}

bool cApp::scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel)
{
	if (nanogui::Screen::scrollEvent(p, rel))
	{
		return true;
	}
	if (mScenario != nullptr)
	{
		return mScenario->ScrollEvent(p, rel);
	}
	return false;
}

void cApp::draw(NVGcontext *ctx)
{
	//Draw the user interface
	Screen::draw(ctx);
}

void cApp::drawContents()
{
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	Update();
	DrawScenario();
}

bool cApp::resizeEvent(const Eigen::Vector2i& size)
{
	bool val = nanogui::Screen::resizeEvent(size);
	if (mScenario != nullptr)
	{
		mScenario->Resize(size);
	}
	return val;
}

double cApp::GetFPS() const
{
	return gFPS;
}

void cApp::BuildScenario(eScene scene)
{
	mScenario.reset();

	switch (scene)
	{
	case eSceneCurve:
		mScenario = std::unique_ptr<cScenario>(new cBirdScenario());
		break;
	case eSceneCharacter:
		mScenario = std::unique_ptr<cScenario>(new cBipedScenario());
		break;
	default:
		assert(false); // unsupported scene
		break;
	}
	
	mScenario->Resize(mSize);
	mScenario->Init();
}

void cApp::UpdateScenario(double time)
{
	double time_elapsed = time - mPrevTime;
	double time_step = 1 / GetFPS();
	time_elapsed = std::min(time_elapsed, time_step);
	StepScenario(time_elapsed);
	mPrevTime = time;
}

void cApp::StepScenario(double time_elapsed)
{
	if (mScenario != nullptr)
	{
		mScenario->Update(time_elapsed);
	}
}

void cApp::Update()
{
	if (EnableAnimation())
	{
		// Advances animation
		UpdateScenario(glfwGetTime());
	}
	UpdateGUI();
}

void cApp::DrawScenario()
{
	if (mScenario != nullptr)
	{
		mScenario->Draw();
	}
}

bool cApp::EnableAnimation() const
{
	bool enable = mEnableAnimation;
	if (mPlayButton != nullptr)
	{
		enable &= mPlayButton->pushed();
	}
	return enable;
}

void cApp::ClearGUI()
{
	if (mGUIWindow != nullptr)
	{
		removeChild(mGUIWindow);
	}
}

void cApp::BuildGUI()
{
	ClearGUI();

	// Build GUI panel that will contain the interface
	mGUIWindow = new nanogui::Window(this, "Scene Control");
	mGUIWindow->setPosition(nanogui::Vector2i(0, 0));
	mGUIWindow->setLayout(new nanogui::GroupLayout());
	
	// Add combo box to choose between difference scenes
	new nanogui::Label(mGUIWindow, "Scene", "sans-bold");
	mSceneCombo = new nanogui::ComboBox(mGUIWindow, {"Bird", "Biped"});
	tComboCallback scene_combo_callback = std::bind(&cApp::SceneComboCallback, this, std::placeholders::_1);
	mSceneCombo->setCallback(scene_combo_callback);
	mSceneCombo->setSelectedIndex(gDefaultScene);

	// Add combo box to pick between different parameter files
	new nanogui::Label(mGUIWindow, "Param File", "sans-bold");
	mParamFileCombo = new nanogui::ComboBox(mGUIWindow);
	tComboCallback param_file_combo_callback = std::bind(&cApp::ParamFileComboCallback, this, std::placeholders::_1);
	mParamFileCombo->setCallback(param_file_combo_callback);
	UpdateParamFileCombo();

	// Add panel for playback control
	new nanogui::Label(mGUIWindow, "Playback", "sans-bold");
	Widget* playback = new Widget(mGUIWindow);
	playback->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
						nanogui::Alignment::Middle, 0, 6));

	// Step backward button
	auto step_backward = new nanogui::Button(playback, "", ENTYPO_ICON_FB);
	step_backward->setCallback(std::bind(&cApp::StepBackwardCallback, this));

	// Play/Pause button
	mPlayButton = new nanogui::Button(playback, "", ENTYPO_ICON_PAUS);
	mPlayButton->setFlags(nanogui::Button::ToggleButton);
	mPlayButton->setPushed(true);
	mPlayButton->setChangeCallback(std::bind(&cApp::TogglePlayCallback, this, std::placeholders::_1));

	// Step forward button
	auto step_forward = new nanogui::Button(playback, "", ENTYPO_ICON_FF);
	step_forward->setCallback(std::bind(&cApp::StepForwardCallback, this));

	// Reload scene button
	auto reload = new nanogui::Button(playback, "", ENTYPO_ICON_CCW);
	reload->setCallback(std::bind(&cApp::Reload, this));

	// Slider to show/control progress of the playback
	mPlaybackSlider = new nanogui::Slider(mGUIWindow);
	mPlaybackSlider->setCallback(std::bind(&cApp::PlaybackSliderCallback, this, std::placeholders::_1));
	mPlaybackSlider->setFinalCallback(std::bind(&cApp::PlaybackSliderFinalCallback, this, std::placeholders::_1));

	// After all GUI has been built, call refresh to reorganize everything
	RefreshGUI();
}

void cApp::UpdateGUI()
{
	if (mScenario != nullptr)
	{
		double progress = mScenario->GetPlaybackProgress();
		mPlaybackSlider->setValue(static_cast<float>(progress));
	}
}

void cApp::RefreshGUI()
{
	performLayout();
}

void cApp::UpdateParamFileCombo()
{
	// update parameter files depending on which scene is active
	const auto& param_files = mScenario->GetParamFiles();
	std::vector<std::string> short_param_files;
	BuildShortFileNames(param_files, short_param_files);
	mParamFileCombo->setItems(param_files, short_param_files);
	mParamFileCombo->setSelectedIndex(0);
}

cApp::eScene cApp::GetCurrScene() const
{
	eScene scene = gDefaultScene;
	if (mSceneCombo != nullptr)
	{
		scene = static_cast<eScene>(mSceneCombo->selectedIndex());
	}
	return scene;
}

const std::string& cApp::GetCurrParamFile() const
{
	int i = mParamFileCombo->selectedIndex();
	const auto& items = mParamFileCombo->items();
	const std::string& param_file = items[i];
	return param_file;
}

void cApp::SceneComboCallback(int i)
{
	eScene scene = static_cast<eScene>(i);
	BuildScenario(scene);
	UpdateParamFileCombo();
	RefreshGUI();
}

void cApp::ParamFileComboCallback(int i)
{
	const std::string& param_file = GetCurrParamFile();
	mScenario->LoadParams(param_file);
	RefreshGUI();
}

void cApp::StepBackwardCallback()
{
	mPlayButton->setPushed(false);
	TogglePlayCallback(false);
	StepScenario(-1 / GetFPS());
}

void cApp::StepForwardCallback()
{
	mPlayButton->setPushed(false);
	TogglePlayCallback(false);
	StepScenario(1 / GetFPS());
}

void cApp::TogglePlayCallback(bool pushed)
{
	if (pushed)
	{
		mPlayButton->setIcon(ENTYPO_ICON_PAUS);
	}
	else
	{
		mPlayButton->setIcon(ENTYPO_ICON_PLAY);
	}
}

void cApp::PlaybackSliderCallback(double val)
{
	if (val >= 1)
	{
		val = 0.999999;
	}

	mScenario->SetPlaybackProgress(val);
	StepScenario(0);
	mEnableAnimation = false;
}

void cApp::PlaybackSliderFinalCallback(double val)
{
	PlaybackSliderCallback(val);
	mEnableAnimation = true;
}

void cApp::Reload()
{
	BuildScenario(GetCurrScene());
	mScenario->LoadParams(GetCurrParamFile());
}

void cApp::BuildShortFileNames(const std::vector<std::string>& files, std::vector<std::string>& out_names) const
{
	// shorten the filepaths for display in the GUI
	out_names.clear();
	std::vector<std::string> short_names;
	for (size_t f = 0; f < files.size(); ++f)
	{
		const std::string& curr_file = files[f];

		int idx = 0;
		for (int i = static_cast<int>(curr_file.size()) - 1; i >= 0; --i)
		{
			char curr_char = curr_file[i];
			if (curr_char == '\\' || curr_char == '/')
			{
				idx = i + 1;
				break;
			}
		}

		std::string filename = curr_file.substr(idx, curr_file.size() - idx);
		out_names.push_back(filename);
	}
}