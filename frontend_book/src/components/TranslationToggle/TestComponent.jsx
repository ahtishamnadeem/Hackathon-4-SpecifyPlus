/**
 * Test component for Urdu Translation Feature
 * This component can be used to test the translation functionality
 */
import React from 'react';
import TranslationToggle from '@site/src/components/TranslationToggle/TranslationToggle';

const TranslationTest = () => {
  const testContent = (
    <div>
      <h1>Test Chapter Content</h1>
      <p>This is a sample chapter content that will be translated to Urdu when the toggle is activated.</p>
      <h2>Technical Concepts</h2>
      <p>ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.</p>
      <ul>
        <li>DDS (Data Distribution Service) for communication</li>
        <li>QoS (Quality of Service) policies for reliability</li>
        <li>Package management and build system</li>
      </ul>
      <h3>Code Example</h3>
      <pre><code>
{`#include <rclcpp/rclcpp.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_node");
  RCLCPP_INFO(node->get_logger(), "Hello, ROS 2!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}`}
      </code></pre>
      <p>The translation should preserve the technical terms like ROS 2, DDS, and QoS in English while translating the rest of the content to Urdu.</p>
    </div>
  );

  return (
    <div style={{ padding: '2rem', maxWidth: '800px', margin: '0 auto' }}>
      <h1>Urdu Translation Feature Test</h1>
      <p>Use the toggle button below to switch between English and Urdu versions of the content.</p>

      <TranslationToggle contentId="test-chapter-1">
        {testContent}
      </TranslationToggle>
    </div>
  );
};

export default TranslationTest;