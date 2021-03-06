#include "TaskDialog.hpp"

#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>
#include <QtGui>
#include <QRadioButton>
#include <QComboBox>



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;



TaskDialog::TaskDialog(QWidget* parent, TaskDescription::Ptr td, std::string wd) :
	QDialog(parent),
	_td(td),
	_changed(false),
	_wd(wd)
{
	//createGUI();
	ui.setupUi(this);
	
	connect(ui.okButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.applyButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.cancelButton, SIGNAL(clicked()), this, SLOT(close()));
	
	updateGUI();
}



void TaskDialog::guiEvent()
{
	QObject* obj = sender();
	
	if (obj == ui.okButton) {
		updateTaskDescription();
		close();
	}
	
	else if (obj == ui.applyButton) {
		updateTaskDescription();
	}
}



void TaskDialog::guiEvent(int index)
{
	QObject* obj = sender();
	
	if (obj == ui.gripperCombo) {
		// we have to find proper frames and update GUI
		
	}
}



/*void TaskDialog::createGUI()
{
	int row = 0;
	
	_okButton = new QPushButton("Ok");
	_applyButton = new QPushButton("Apply");
	_cancelButton = new QPushButton("Cancel");
	
	QGroupBox* targetBox = new QGroupBox("Target");
	_targetCombo = new QComboBox;
	
	QGroupBox* gripperBox = new QGroupBox("Gripper");
	QLabel* gripperLabel = new QLabel("Device");
	_gripperCombo = new QComboBox;
	
	QGroupBox* resultBox = new QGroupBox("Evaluation parameters");
	QLabel* baseLabel = new QLabel("Baseline");
	QLabel* weightLabel = new QLabel("Weights");
	QLabel* shapeLabel = new QLabel("Shape");
	QLabel* coverageLabel = new QLabel("Coverage");
	QLabel* successLabel = new QLabel("Success");
	QLabel* wrenchLabel = new QLabel("Wrench");
	_baseShapeEdit = new QLineEdit("0"); _baseShapeEdit->setDisabled(true);
	_baseCoverageEdit = new QLineEdit("1");
	_baseSuccessEdit = new QLineEdit("1");
	_baseWrenchEdit = new QLineEdit("1");
	_weightShapeEdit = new QLineEdit("0"); _weightShapeEdit->setDisabled(true);
	_weightCoverageEdit = new QLineEdit("1");
	_weightSuccessEdit = new QLineEdit("1");
	_weightWrenchEdit = new QLineEdit("1");
	
	QGroupBox* distanceBox = new QGroupBox("Distance parameters");
	QLabel* teachDistLabel = new QLabel("Teach distance");
	QLabel* preDistLabel = new QLabel("Prefiltering distance");
	QLabel* coverageDistLabel = new QLabel("Coverage distance");
	_teachDistEdit = new QLineEdit("");
	_preDistEdit = new QLineEdit("");
	_coverageDistEdit = new QLineEdit("");
	
	QGroupBox* interferenceBox = new QGroupBox("Interference objects");
	
	QGroupBox* limitBox = new QGroupBox("Limits");
	QLabel* intLimitLabel = new QLabel("INT limit");
	QLabel* wreLimitLabel = new QLabel("WRE limit");
	_intLimitEdit = new QLineEdit("0.0");
	_wreLimitEdit = new QLineEdit("0.0");
	
	connect(_okButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_applyButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_cancelButton, SIGNAL(clicked()), this, SLOT(close()));
	
	QGridLayout* layout = new QGridLayout;
	QHBoxLayout* buttonLayout = new QHBoxLayout;
	QGridLayout* targetLayout = new QGridLayout;
	QGridLayout* gripperLayout = new QGridLayout;
	QGridLayout* resultLayout = new QGridLayout;
	QGridLayout* distLayout = new QGridLayout;
	QGridLayout* limitLayout = new QGridLayout;
	QGridLayout* interferenceLayout = new QGridLayout;
	
	buttonLayout->addWidget(_okButton);
	buttonLayout->addWidget(_applyButton);
	buttonLayout->addWidget(_cancelButton);
	buttonLayout->addStretch(1);
	
	row = 0;
	targetLayout->addWidget(_targetCombo, row++, 0);
	
	row = 0;
	gripperLayout->addWidget(gripperLabel, row, 0);
	gripperLayout->addWidget(_gripperCombo, row++, 1);
	
	row = 0;
	distLayout->addWidget(teachDistLabel, row, 0);
	distLayout->addWidget(_teachDistEdit, row++, 1);
	distLayout->addWidget(preDistLabel, row, 0);
	distLayout->addWidget(_preDistEdit, row++, 1);
	distLayout->addWidget(coverageDistLabel, row, 0);
	distLayout->addWidget(_coverageDistEdit, row++, 1);
	
	row = 0;
	//resultLayout->addWidget(shapeLabel, row, 1);
	resultLayout->addWidget(coverageLabel, row, 1);
	resultLayout->addWidget(successLabel, row, 2);
	resultLayout->addWidget(wrenchLabel, row++, 3);
	resultLayout->addWidget(baseLabel, row, 0);
	//resultLayout->addWidget(_baseShapeEdit, row, 1);
	resultLayout->addWidget(_baseCoverageEdit, row, 1);
	resultLayout->addWidget(_baseSuccessEdit, row, 2);
	resultLayout->addWidget(_baseWrenchEdit, row++, 3);
	resultLayout->addWidget(weightLabel, row, 0);
	//resultLayout->addWidget(_weightShapeEdit, row, 1);
	resultLayout->addWidget(_weightCoverageEdit, row, 1);
	resultLayout->addWidget(_weightSuccessEdit, row, 2);
	resultLayout->addWidget(_weightWrenchEdit, row++, 3);
	
	row = 0;
	limitLayout->addWidget(intLimitLabel, row, 0);
	limitLayout->addWidget(_intLimitEdit, row++, 1);
	limitLayout->addWidget(wreLimitLabel, row, 0);
	limitLayout->addWidget(_wreLimitEdit, row++, 1);
	
	row = 0;
	layout->addWidget(targetBox, row++, 0);
	layout->addWidget(gripperBox, row, 0);
	layout->addWidget(distanceBox, row++, 1);
	layout->addWidget(interferenceBox, row, 0);
	layout->addWidget(limitBox, row++, 1);
	layout->addWidget(resultBox, row++, 0, 1, 2);
	layout->addLayout(buttonLayout, row++, 0);
	
	targetBox->setLayout(targetLayout);
	gripperBox->setLayout(gripperLayout);
	distanceBox->setLayout(distLayout);
	resultBox->setLayout(resultLayout);
	interferenceBox->setLayout(interferenceLayout);
	limitBox->setLayout(limitLayout);
	setLayout(layout);
}*/



void TaskDialog::updateGUI()
{
	// fill the target box
	ui.targetCombo->clear();
	for(Object::Ptr obj : _td->getWorkCell()->getObjects()) {
		ui.targetCombo->addItem(QString::fromStdString(obj->getName()));
	}
	ui.targetCombo->setCurrentIndex(ui.targetCombo->findText(QString::fromStdString(_td->getTargetObject()->getName())));
	
	// fill the gripper box
	ui.gripperCombo->clear();
	for(Device::Ptr dev : _td->getWorkCell()->getDevices()) {
		ui.gripperCombo->addItem(QString::fromStdString(dev->getName()));
	}
	ui.gripperCombo->setCurrentIndex(ui.gripperCombo->findText(QString::fromStdString(_td->getGripperDevice()->getName())));
	
	// update the distances section
	Q teachDist = _td->getTeachDistance();
	teachDist(3) *= Rad2Deg;
	teachDist(4) *= Rad2Deg;
	stringstream ss;
	ss << teachDist;
	ui.teachDistEdit->setText(QString::fromStdString(ss.str()));
	
	Q preDist = _td->getPrefilteringDistance();
	preDist(1) *= Rad2Deg;
	preDist(2) *= Rad2Deg;
	ss.str("");
	ss << preDist;
	ui.preDistEdit->setText(QString::fromStdString(ss.str()));
	
	Q covDist = _td->getCoverageDistance();
	//covDist(6) *= Rad2Deg;
	covDist(1) *= Rad2Deg;
	covDist(2) *= Rad2Deg;
	ss.str("");
	ss << covDist;
	ui.coverageDistEdit->setText(QString::fromStdString(ss.str()));
	
	// update the limits section
	ui.intLimitEdit->setText(QString::number(_td->getInterferenceLimit()));
	ui.wreLimitEdit->setText(QString::number(_td->getWrenchLimit()));
	
	// update the result section
	//ui.baseShapeEdit->setText(QString::number(_td->getBaseline().shape));
	ui.baseCoverageEdit->setText(QString::number(_td->getBaseline().coverage));
	ui.baseSuccessEdit->setText(QString::number(_td->getBaseline().success));
	ui.baseWrenchEdit->setText(QString::number(_td->getBaseline().wrench));
	//_weightShapeEdit->setText(QString::number(_td->getWeights().shape));
	ui.weightCoverageEdit->setText(QString::number(_td->getWeights().coverage));
	ui.weightSuccessEdit->setText(QString::number(_td->getWeights().success));
	ui.weightWrenchEdit->setText(QString::number(_td->getWeights().wrench));
}



void TaskDialog::updateTaskDescription()
{
	// set new target
	_td->setTarget(ui.targetCombo->currentText().toStdString());
	
	// update distances
	Q teachDist;
	stringstream ss(ui.teachDistEdit->text().toStdString());
	ss >> teachDist;
	teachDist(3) *= Deg2Rad;
	teachDist(4) *= Deg2Rad;
	_td->setTeachDistance(teachDist);
	
	Q preDist;
	ss.str(ui.preDistEdit->text().toStdString());
	ss >> preDist;
	//covDist(6) *= Deg2Rad;
	preDist(1) *= Deg2Rad;
	preDist(2) *= Deg2Rad;
	_td->setPrefilteringDistance(preDist);
	
	Q covDist;
	ss.str(ui.coverageDistEdit->text().toStdString());
	ss >> covDist;
	//covDist(6) *= Deg2Rad;
	covDist(1) *= Deg2Rad;
	covDist(2) *= Deg2Rad;
	_td->setCoverageDistance(covDist);
	
	// update limits
	_td->setInterferenceLimit(ui.intLimitEdit->text().toDouble());
	_td->setWrenchLimit(ui.wreLimitEdit->text().toDouble());
	
	// update baseline & weights
	TaskDescription::Qualities& base = _td->getBaseline();
	//base.shape = _baseShapeEdit->text().toDouble();
	base.coverage = ui.baseCoverageEdit->text().toDouble();
	base.success = ui.baseSuccessEdit->text().toDouble();
	base.wrench = ui.baseWrenchEdit->text().toDouble();
	
	TaskDescription::Qualities& w = _td->getWeights();
	//w.shape = _weightShapeEdit->text().toDouble();
	w.coverage = ui.weightCoverageEdit->text().toDouble();
	w.success = ui.weightSuccessEdit->text().toDouble();
	w.wrench = ui.weightWrenchEdit->text().toDouble();
}
