#!/bin/bash
# Deployment script for ROS 2 Fundamentals Book

set -e  # Exit on any error

echo "🚀 Starting deployment process for ROS 2 Fundamentals Book..."

# Build the site
echo "🏗️  Building the site..."
npm run build

if [ $? -eq 0 ]; then
    echo "✅ Build completed successfully!"
    echo "📁 Static files are in the 'build' directory"
    echo ""
    echo "🎉 Deployment ready!"
    echo "The site can be served from the 'build' directory"
    echo ""
    echo "To serve locally for testing:"
    echo "  cd frontend_book && npx docusaurus serve"
    echo ""
    echo "To deploy to your hosting provider:"
    echo "  Upload contents of 'build' directory to your web server"
else
    echo "❌ Build failed!"
    exit 1
fi