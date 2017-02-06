#pragma once
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/button.h>
#include <nanogui/combobox.h>
#include <nanogui/slider.h>

#include <iostream>
#include <string>
#include <memory>
#include <functional>

#include "scenarios/Scenario.h"

class cApp : public nanogui::Screen {
public:
	enum eScene
	{
		eSceneCurve,
		eSceneCharacter,
		eSceneMax
	};
	
	cApp(int w, int h, const std::string& title);
	virtual ~cApp();

	virtual void Init();

	virtual bool keyboardEvent(int key, int scancode, int action, int modifiers);
	virtual bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers);
	virtual bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers);
	virtual bool scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel);

	virtual void draw(NVGcontext *ctx);
	virtual void drawContents();
	
	/// Set window size
	virtual bool resizeEvent(const Eigen::Vector2i& size);

	virtual double GetFPS() const;

protected:
	
	typedef std::function<void(int)> tComboCallback;
	typedef std::function<void()> tButtonCallback;
	typedef std::function<void(bool)> tToggleCallback;
	typedef std::function<void(double)> tSliderCallback;

	std::unique_ptr<cScenario> mScenario;
	nanogui::Window* mGUIWindow;
	nanogui::ComboBox* mSceneCombo;
	nanogui::ComboBox* mParamFileCombo;
	nanogui::Button* mPlayButton;
	nanogui::Slider* mPlaybackSlider;

	double mPrevTime;
	bool mEnableAnimation;

	virtual void BuildScenario(eScene scene);
	virtual void UpdateScenario(double time);
	virtual void StepScenario(double time_elapsed);

	virtual void Update();
	virtual void DrawScenario();

	virtual bool EnableAnimation() const;

	virtual void ClearGUI();
	virtual void BuildGUI();
	virtual void UpdateGUI();
	virtual void RefreshGUI();
	virtual void UpdateParamFileCombo();
	virtual eScene GetCurrScene() const;
	virtual const std::string& GetCurrParamFile() const;

	virtual void SceneComboCallback(int i);
	virtual void ParamFileComboCallback(int i);
	virtual void StepBackwardCallback();
	virtual void StepForwardCallback();
	virtual void TogglePlayCallback(bool pushed);
	virtual void PlaybackSliderCallback(double val);
	virtual void PlaybackSliderFinalCallback(double val);

	virtual void Reload();

	virtual void BuildShortFileNames(const std::vector<std::string>& files, std::vector<std::string>& out_names) const;
};