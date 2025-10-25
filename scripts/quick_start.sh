#!/bin/bash

# Quick Start Script for Flask LiDAR App
# This script automates the initial setup process

set -e  # Exit on error

echo "🚀 Flask LiDAR App - Quick Start Setup"
echo "======================================"
echo ""

# Check Python version
echo "📋 Checking Python version..."
python3 --version || { echo "❌ Python 3 is required but not installed."; exit 1; }

# Create virtual environment
echo ""
echo "📦 Creating virtual environment..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo "✅ Virtual environment created"
else
    echo "⚠️  Virtual environment already exists"
fi

# Activate virtual environment
echo ""
echo "🔧 Activating virtual environment..."
source venv/bin/activate || { echo "❌ Failed to activate virtual environment"; exit 1; }

# Install dependencies
echo ""
echo "📥 Installing dependencies..."
pip install --upgrade pip
pip install -r requirements.txt
echo "✅ Dependencies installed"

# Check for .env file
echo ""
echo "🔐 Checking environment configuration..."
if [ ! -f ".env" ]; then
    if [ -f ".env.example" ]; then
        cp .env.example .env
        echo "⚠️  Created .env file from template"
        echo ""
        echo "⚠️  IMPORTANT: Please edit .env and add your:"
        echo "   1. GOOGLE_API_KEY (get from https://makersuite.google.com/app/apikey)"
        echo "   2. SECRET_KEY (generate with: python -c 'import secrets; print(secrets.token_hex(32))')"
        echo ""
        echo "Generate SECRET_KEY now? (y/n)"
        read -r response
        if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
            SECRET_KEY=$(python -c "import secrets; print(secrets.token_hex(32))")
            echo ""
            echo "Generated SECRET_KEY:"
            echo "$SECRET_KEY"
            echo ""
            echo "Copy this key and paste it in your .env file"
        fi
        echo ""
        echo "❌ Setup incomplete. Please configure .env file and run this script again."
        exit 1
    else
        echo "❌ .env.example not found. Please create .env manually."
        exit 1
    fi
else
    echo "✅ .env file exists"
    
    # Check if required variables are set
    source .env
    if [ -z "$GOOGLE_API_KEY" ] || [ "$GOOGLE_API_KEY" = "your-google-api-key-here" ]; then
        echo "❌ GOOGLE_API_KEY not configured in .env"
        echo "   Get your key from: https://makersuite.google.com/app/apikey"
        exit 1
    fi
    
    if [ -z "$SECRET_KEY" ] || [ "$SECRET_KEY" = "your-secret-key-here" ]; then
        echo "❌ SECRET_KEY not configured in .env"
        echo "   Generate one with: python -c 'import secrets; print(secrets.token_hex(32))'"
        exit 1
    fi
    
    echo "✅ Environment variables configured"
fi

# Initialize database
echo ""
echo "🗄️  Initializing database..."
python3 << END
from app import app, db
with app.app_context():
    db.create_all()
    print("✅ Database initialized successfully!")
END

# Create uploads directory
echo ""
echo "📁 Creating uploads directory..."
mkdir -p uploads
echo "✅ Uploads directory ready"

# Display success message
echo ""
echo "======================================"
echo "✅ Setup Complete!"
echo "======================================"
echo ""
echo "🎉 Your Flask LiDAR app is ready to run!"
echo ""
echo "To start the server:"
echo "  1. source venv/bin/activate"
echo "  2. python app.py"
echo ""
echo "The server will be available at:"
echo "  http://localhost:5000"
echo ""
echo "To find your local IP for iOS connection:"
echo "  ifconfig | grep 'inet ' | grep -v 127.0.0.1"
echo ""
echo "Configure your iOS app with:"
echo "  http://YOUR_IP_ADDRESS:5000"
echo ""
